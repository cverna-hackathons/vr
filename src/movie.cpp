#include "movie.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/channel_layout.h>
#include <libavutil/opt.h>
#include <libswresample/swresample.h>
#include <libswscale/swscale.h>
}

namespace {
constexpr int kRate = 48000;
constexpr size_t kRingMax = size_t(kRate) * 2 * 1;     // 1 s of stereo samples
constexpr size_t kQueueBytes = 48u * 1024 * 1024;
void sleep_ms(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }
}  // namespace

int SDLCALL MoviePlayer::on_event(void* user, SDL_Event* e) {
    auto* self = static_cast<MoviePlayer*>(user);
    if (e->type == SDL_AUDIODEVICEREMOVED && !e->adevice.iscapture && e->adevice.which == self->device_id_.load()) self->removed_ = true;
    return 0;
}

void MoviePlayer::drop_device() {
    if (!dev_) return;
    SDL_DelEventWatch(&MoviePlayer::on_event, this);
    SDL_CloseAudioDevice(dev_);
    dev_ = 0;
    device_id_ = 0;
}

bool MoviePlayer::open_device() {
    SDL_AudioSpec want{}, have{};
    want.freq = kRate;
    want.format = AUDIO_S16SYS;
    want.channels = 2;
    want.samples = 1024;
    want.callback = &MoviePlayer::audio_callback;
    want.userdata = this;
    dev_ = SDL_OpenAudioDevice(nullptr, 0, &want, &have, 0);
    if (!dev_) return false;
    device_id_ = dev_;
    removed_ = false;
    SDL_AddEventWatch(&MoviePlayer::on_event, this);
    seen_callbacks_ = callbacks_.load();
    last_progress_ms_ = SDL_GetTicks();
    return true;
}

void MoviePlayer::service() {
    if (paused_ || quit_) { last_progress_ms_ = SDL_GetTicks(); return; }   // no callbacks are expected while paused
    const uint32_t now = SDL_GetTicks();
    if (dev_) {
        const uint64_t c = callbacks_.load(std::memory_order_relaxed);
        const bool disconnected = removed_.exchange(false);
        if (c != seen_callbacks_) { seen_callbacks_ = c; last_progress_ms_ = now; }
        if (!disconnected && now - last_progress_ms_ < 1200) return;
        std::fprintf(stderr, "Movie: the audio output %s (another application took it, or the device changed). Reconnecting...\n",
                     disconnected ? "was disconnected" : "stopped");
        drop_device();
        last_attempt_ms_ = 0;
    }
    if (now - last_attempt_ms_ < 2000) return;
    last_attempt_ms_ = now;
    if (open_device()) {
        SDL_PauseAudioDevice(dev_, 0);
        std::printf("Movie: audio output restored.\n");
    }
}

bool MoviePlayer::open(const std::string& path, int max_width) {
    close();
    path_ = path;
    if (avformat_open_input(&fmt_, path.c_str(), nullptr, nullptr) < 0) {
        std::fprintf(stderr, "Movie: cannot open %s\n", path.c_str());
        return false;
    }
    avformat_find_stream_info(fmt_, nullptr);
    vidx_ = av_find_best_stream(fmt_, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
    aidx_ = av_find_best_stream(fmt_, AVMEDIA_TYPE_AUDIO, -1, vidx_, nullptr, 0);
    if (vidx_ < 0) { std::fprintf(stderr, "Movie: no video stream\n"); avformat_close_input(&fmt_); return false; }
    has_audio_ = aidx_ >= 0;
    vtb_ = av_q2d(fmt_->streams[vidx_]->time_base);
    if (has_audio_) atb_ = av_q2d(fmt_->streams[aidx_]->time_base);
    duration_ = fmt_->duration > 0 ? double(fmt_->duration) / AV_TIME_BASE : 0;
    const int sw = fmt_->streams[vidx_]->codecpar->width, sh = fmt_->streams[vidx_]->codecpar->height;
    w_ = (sw < max_width ? sw : max_width) & ~1;
    h_ = int(double(sh) * w_ / sw) & ~1;
    frame_.assign(size_t(w_) * h_ * 3, 0);

    if (!open_device()) std::fprintf(stderr, "Movie: no audio device (%s); playing silent, will keep trying.\n", SDL_GetError());

    quit_ = false;
    serial_ = 0;
    seek_req_ = -1.0;
    target_pts_ = 0.0;
    clock_ok_ = !has_audio_;  // without an audio stream the clock starts immediately (silent sample counting)
    base_pts_ = 0;
    played_ = 0;
    t_demux_ = std::thread(&MoviePlayer::demux_thread, this);
    if (has_audio_) t_audio_ = std::thread(&MoviePlayer::audio_thread, this);
    t_video_ = std::thread(&MoviePlayer::video_thread, this);
    if (dev_) SDL_PauseAudioDevice(dev_, 0);
    std::printf("Movie: %s  %dx%d (shown at %dx%d), %.0f s, audio %s\n", path.c_str(), sw, sh, w_, h_, duration_,
                has_audio_ ? "yes" : "no");
    return true;
}

void MoviePlayer::close() {
    quit_ = true;
    vq_.cv.notify_all();
    aq_.cv.notify_all();
    if (dev_) SDL_PauseAudioDevice(dev_, 1);
    if (t_demux_.joinable()) t_demux_.join();
    if (t_audio_.joinable()) t_audio_.join();
    if (t_video_.joinable()) t_video_.join();
    drop_device();
    clear(vq_);
    clear(aq_);
    if (fmt_) avformat_close_input(&fmt_);
}

bool MoviePlayer::take_frame(std::vector<uint8_t>& rgb) {
    std::lock_guard<std::mutex> lock(fmu_);
    if (!fresh_) return false;
    rgb = frame_;
    fresh_ = false;
    return true;
}

bool MoviePlayer::clock_valid() {
    std::lock_guard<std::mutex> lock(amu_);
    return clock_ok_;
}

double MoviePlayer::position() {
    std::lock_guard<std::mutex> lock(amu_);
    return base_pts_ + double(played_) / kRate;
}

void MoviePlayer::toggle_pause() {
    paused_ = !paused_;
    if (dev_) SDL_PauseAudioDevice(dev_, paused_ ? 1 : 0);
}

void MoviePlayer::seek_relative(double seconds) {
    double t = position() + seconds;
    if (t < 0) t = 0;
    if (duration_ > 0 && t > duration_ - 1.0) t = duration_ - 1.0;
    seek_req_ = t;
}

// ---- queues ----------------------------------------------------------------------------------

void MoviePlayer::push(Queue& q, Item it) {
    std::lock_guard<std::mutex> lock(q.m);
    if (it.pkt) q.bytes += size_t(it.pkt->size);
    q.q.push_back(it);
    q.cv.notify_one();
}

bool MoviePlayer::pop(Queue& q, Item& out) {
    std::unique_lock<std::mutex> lock(q.m);
    while (q.q.empty()) {
        if (quit_) return false;
        q.cv.wait_for(lock, std::chrono::milliseconds(50));
    }
    out = q.q.front();
    q.q.pop_front();
    if (out.pkt) q.bytes -= size_t(out.pkt->size);
    q.cv.notify_all();
    return true;
}

void MoviePlayer::clear(Queue& q) {
    std::lock_guard<std::mutex> lock(q.m);
    for (Item& it : q.q) if (it.pkt) av_packet_free(&it.pkt);
    q.q.clear();
    q.bytes = 0;
    q.cv.notify_all();
}

// ---- demux -----------------------------------------------------------------------------------

void MoviePlayer::demux_thread() {
    AVPacket* pkt = av_packet_alloc();
    bool eof = false;
    while (!quit_) {
        const double seek = seek_req_.exchange(-1.0);
        if (seek >= 0) {
            av_seek_frame(fmt_, -1, int64_t(seek * AV_TIME_BASE), AVSEEK_FLAG_BACKWARD);
            ++serial_;
            clear(vq_);
            clear(aq_);
            target_pts_ = seek;
            {
                std::lock_guard<std::mutex> lock(amu_);
                ring_.clear();
                rpos_ = 0;
                played_ = 0;
                base_pts_ = seek;
                clock_ok_ = !has_audio_;  // with audio, the first decoded audio frame re-establishes the clock
            }
            eof = false;
        }
        if (eof) { sleep_ms(20); continue; }
        // Back-pressure, with a generous cap so uneven interleaving can't starve the other stream.
        if (vq_.bytes > kQueueBytes || aq_.bytes > kQueueBytes) { sleep_ms(5); continue; }
        if (av_read_frame(fmt_, pkt) < 0) {
            push(vq_, {nullptr, serial_});
            if (has_audio_) push(aq_, {nullptr, serial_});
            eof = true;
            continue;
        }
        if (pkt->stream_index == vidx_ || pkt->stream_index == aidx_) {
            AVPacket* c = av_packet_alloc();
            av_packet_move_ref(c, pkt);
            push(c->stream_index == vidx_ ? vq_ : aq_, {c, serial_});
        } else {
            av_packet_unref(pkt);
        }
    }
    av_packet_free(&pkt);
}

// ---- audio -----------------------------------------------------------------------------------

void MoviePlayer::audio_thread() {
    const AVCodecParameters* par = fmt_->streams[aidx_]->codecpar;
    const AVCodec* codec = avcodec_find_decoder(par->codec_id);
    AVCodecContext* ctx = avcodec_alloc_context3(codec);
    avcodec_parameters_to_context(ctx, par);
    if (avcodec_open2(ctx, codec, nullptr) < 0) {
        std::fprintf(stderr, "Movie: cannot open audio decoder; playing silent.\n");
        avcodec_free_context(&ctx);
        std::lock_guard<std::mutex> lock(amu_);
        has_audio_ = false;
        clock_ok_ = true;
        return;
    }
    AVFrame* fr = av_frame_alloc();
    SwrContext* swr = nullptr;
    int serial = 0;
    bool first = true;
    double next_pts = 0;
    std::vector<int16_t> out;
    Item it;
    while (pop(aq_, it)) {
        if (it.serial != serial) {
            avcodec_flush_buffers(ctx);
            swr_free(&swr);
            serial = it.serial;
            first = true;
        }
        if (!it.pkt) continue;
        if (it.serial != serial_) { av_packet_free(&it.pkt); continue; }
        if (avcodec_send_packet(ctx, it.pkt) == 0) {
            while (!quit_ && avcodec_receive_frame(ctx, fr) == 0) {
                double pts = fr->best_effort_timestamp == AV_NOPTS_VALUE ? next_pts : fr->best_effort_timestamp * atb_;
                const double dur = double(fr->nb_samples) / fr->sample_rate;
                next_pts = pts + dur;
                if (pts + dur < target_pts_) continue;  // before the seek target
                if (!swr) {
                    AVChannelLayout stereo;
                    av_channel_layout_default(&stereo, 2);
                    swr_alloc_set_opts2(&swr, &stereo, AV_SAMPLE_FMT_S16, kRate, &fr->ch_layout,
                                        AVSampleFormat(fr->format), fr->sample_rate, 0, nullptr);
                    if (!swr || swr_init(swr) < 0) { swr_free(&swr); continue; }
                }
                const int cap = swr_get_out_samples(swr, fr->nb_samples);
                out.resize(size_t(cap) * 2);
                uint8_t* dst[1] = {reinterpret_cast<uint8_t*>(out.data())};
                const int n = swr_convert(swr, dst, cap, const_cast<const uint8_t**>(fr->extended_data), fr->nb_samples);
                if (n <= 0) continue;
                // Don't let a backlog build up: wait while the ring is full (unless a seek happened).
                while (!quit_ && it.serial == serial_) {
                    {
                        std::lock_guard<std::mutex> lock(amu_);
                        if (ring_.size() - rpos_ < kRingMax) break;
                    }
                    sleep_ms(5);
                }
                if (it.serial != serial_) break;
                std::lock_guard<std::mutex> lock(amu_);
                if (first) {
                    ring_.clear();
                    rpos_ = 0;
                    played_ = 0;
                    base_pts_ = pts;
                    clock_ok_ = true;
                    first = false;
                }
                ring_.insert(ring_.end(), out.begin(), out.begin() + size_t(n) * 2);
            }
        }
        av_packet_free(&it.pkt);
    }
    swr_free(&swr);
    av_frame_free(&fr);
    avcodec_free_context(&ctx);
}

void MoviePlayer::audio_callback(void* user, Uint8* stream, int len) {
    static_cast<MoviePlayer*>(user)->callbacks_.fetch_add(1, std::memory_order_relaxed);
    static_cast<MoviePlayer*>(user)->fill_audio(stream, len);
}

void MoviePlayer::fill_audio(Uint8* stream, int len) {
    int16_t* dst = reinterpret_cast<int16_t*>(stream);
    const size_t want = size_t(len) / sizeof(int16_t);
    const float vol = volume_;
    std::lock_guard<std::mutex> lock(amu_);
    size_t avail = ring_.size() - rpos_;
    size_t n = std::min(want, avail & ~size_t(1));
    for (size_t i = 0; i < n; ++i) {
        const float v = float(ring_[rpos_ + i]) * vol;
        dst[i] = int16_t(v > 32767.f ? 32767.f : (v < -32768.f ? -32768.f : v));
    }
    rpos_ += n;
    if (n < want) std::memset(dst + n, 0, (want - n) * sizeof(int16_t));
    if (has_audio_) played_ += n / 2;
    else if (clock_ok_) played_ += want / 2;  // no audio stream: the silent device is the clock
    if (rpos_ > (1u << 16)) { ring_.erase(ring_.begin(), ring_.begin() + long(rpos_)); rpos_ = 0; }
}

// ---- video -----------------------------------------------------------------------------------

void MoviePlayer::video_thread() {
    const AVCodecParameters* par = fmt_->streams[vidx_]->codecpar;
    const AVCodec* codec = avcodec_find_decoder(par->codec_id);
    AVCodecContext* ctx = avcodec_alloc_context3(codec);
    avcodec_parameters_to_context(ctx, par);
    ctx->thread_count = 3;
    if (avcodec_open2(ctx, codec, nullptr) < 0) {
        std::fprintf(stderr, "Movie: cannot open video decoder\n");
        avcodec_free_context(&ctx);
        return;
    }
    SwsContext* sws = sws_getContext(ctx->width, ctx->height, ctx->pix_fmt, w_, h_, AV_PIX_FMT_RGB24,
                                     SWS_BILINEAR, nullptr, nullptr, nullptr);
    AVFrame* fr = av_frame_alloc();
    std::vector<uint8_t> rgb(size_t(w_) * h_ * 3);
    int serial = 0;
    Item it;
    while (pop(vq_, it)) {
        if (it.serial != serial) { avcodec_flush_buffers(ctx); serial = it.serial; }
        if (!it.pkt) continue;
        if (it.serial != serial_) { av_packet_free(&it.pkt); continue; }
        if (avcodec_send_packet(ctx, it.pkt) == 0) {
            while (!quit_ && avcodec_receive_frame(ctx, fr) == 0) {
                const double pts = fr->best_effort_timestamp == AV_NOPTS_VALUE ? 0 : fr->best_effort_timestamp * vtb_;
                if (it.serial != serial_) break;
                if (pts < target_pts_ - 0.02) continue;  // decoded only to reach the seek target
                // Wait until the audio clock reaches this frame (the clock stalls while paused).
                bool late = false;
                while (!quit_ && it.serial == serial_) {
                    if (clock_valid()) {
                        const double lag = position() - pts;
                        if (lag >= 0) { late = lag > 0.04; break; }
                    }
                    sleep_ms(2);
                }
                if (quit_ || it.serial != serial_) break;
                if (late && !paused_) continue;  // behind: skip conversion to catch up
                uint8_t* dst[1] = {rgb.data()};
                int stride[1] = {w_ * 3};
                sws_scale(sws, fr->data, fr->linesize, 0, ctx->height, dst, stride);
                std::lock_guard<std::mutex> lock(fmu_);
                frame_.swap(rgb);
                fresh_ = true;
            }
        }
        av_packet_free(&it.pkt);
    }
    av_frame_free(&fr);
    sws_freeContext(sws);
    avcodec_free_context(&ctx);
}
