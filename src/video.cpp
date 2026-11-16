#include "video.hpp"

#include <chrono>
#include <cstdio>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

bool VideoPlayer::start(const std::string& path, int max_width) {
    stop();
    path_ = path;
    AVFormatContext* fmt = nullptr;
    if (avformat_open_input(&fmt, path.c_str(), nullptr, nullptr) < 0) {
        std::fprintf(stderr, "Video: cannot open %s\n", path.c_str());
        return false;
    }
    avformat_find_stream_info(fmt, nullptr);
    const int idx = av_find_best_stream(fmt, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
    if (idx < 0) { std::fprintf(stderr, "Video: no video stream\n"); avformat_close_input(&fmt); return false; }
    const AVCodecParameters* par = fmt->streams[idx]->codecpar;
    src_w_ = par->width;
    src_h_ = par->height;
    avformat_close_input(&fmt);
    if (src_w_ <= 0 || src_h_ <= 0) return false;
    w_ = src_w_ < max_width ? src_w_ : max_width;
    h_ = int(double(src_h_) * w_ / src_w_);
    w_ &= ~1;
    h_ &= ~1;
    frame_.assign(size_t(w_) * h_ * 3, 0);
    quit_ = false;
    thread_ = std::thread(&VideoPlayer::run, this);
    std::printf("Video: %s (%dx%d, shown at %dx%d)\n", path.c_str(), src_w_, src_h_, w_, h_);
    return true;
}

void VideoPlayer::stop() {
    quit_ = true;
    if (thread_.joinable()) thread_.join();
}

bool VideoPlayer::take_frame(std::vector<uint8_t>& rgb) {
    std::lock_guard<std::mutex> lock(mu_);
    if (!fresh_) return false;
    rgb = frame_;
    fresh_ = false;
    return true;
}

void VideoPlayer::run() {
    using clock = std::chrono::steady_clock;
    AVFormatContext* fmt = nullptr;
    if (avformat_open_input(&fmt, path_.c_str(), nullptr, nullptr) < 0) return;
    avformat_find_stream_info(fmt, nullptr);
    const int idx = av_find_best_stream(fmt, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
    AVStream* st = fmt->streams[idx];
    const AVCodec* codec = avcodec_find_decoder(st->codecpar->codec_id);
    AVCodecContext* ctx = avcodec_alloc_context3(codec);
    avcodec_parameters_to_context(ctx, st->codecpar);
    ctx->thread_count = 2;  // leave CPU for tracking and rendering
    if (avcodec_open2(ctx, codec, nullptr) < 0) {
        std::fprintf(stderr, "Video: cannot open decoder\n");
        avcodec_free_context(&ctx);
        avformat_close_input(&fmt);
        return;
    }
    SwsContext* sws = sws_getContext(ctx->width, ctx->height, ctx->pix_fmt, w_, h_, AV_PIX_FMT_RGB24,
                                     SWS_BILINEAR, nullptr, nullptr, nullptr);
    AVPacket* pkt = av_packet_alloc();
    AVFrame* fr = av_frame_alloc();
    std::vector<uint8_t> rgb(size_t(w_) * h_ * 3);
    const double tb = av_q2d(st->time_base);
    auto t0 = clock::now();
    double base_pts = -1;  // pts (s) corresponding to t0

    while (!quit_) {
        const int r = av_read_frame(fmt, pkt);
        if (r < 0) {  // end of file: loop
            av_seek_frame(fmt, idx, 0, AVSEEK_FLAG_BACKWARD);
            avcodec_flush_buffers(ctx);
            base_pts = -1;
            continue;
        }
        if (pkt->stream_index == idx && avcodec_send_packet(ctx, pkt) == 0) {
            while (!quit_ && avcodec_receive_frame(ctx, fr) == 0) {
                const double pts = (fr->best_effort_timestamp == AV_NOPTS_VALUE ? 0 : fr->best_effort_timestamp) * tb;
                if (base_pts < 0) { base_pts = pts; t0 = clock::now(); }
                const auto due = t0 + std::chrono::duration_cast<clock::duration>(std::chrono::duration<double>(pts - base_pts));
                const auto now = clock::now();
                if (now > due + std::chrono::milliseconds(250)) {  // far behind: drop and resync
                    base_pts = pts;
                    t0 = now;
                } else if (now > due + std::chrono::milliseconds(40)) {
                    continue;  // slightly behind: skip conversion to catch up
                } else {
                    while (!quit_ && clock::now() < due) std::this_thread::sleep_for(std::chrono::milliseconds(2));
                }
                uint8_t* dst[1] = {rgb.data()};
                int stride[1] = {w_ * 3};
                sws_scale(sws, fr->data, fr->linesize, 0, ctx->height, dst, stride);
                std::lock_guard<std::mutex> lock(mu_);
                frame_.swap(rgb);
                fresh_ = true;
            }
        }
        av_packet_unref(pkt);
    }
    av_frame_free(&fr);
    av_packet_free(&pkt);
    sws_freeContext(sws);
    avcodec_free_context(&ctx);
    avformat_close_input(&fmt);
}
