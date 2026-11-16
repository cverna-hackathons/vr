#include "xcapture.hpp"

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/extensions/XShm.h>
#include <X11/extensions/Xfixes.h>
#include <X11/extensions/Xrandr.h>
#include <sys/ipc.h>
#include <sys/shm.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>

bool find_output_rect(const std::string& name, XRect& out, std::string& found_name) {
    Display* dpy = XOpenDisplay(nullptr);
    if (!dpy) return false;
    bool ok = false;
    XRRScreenResources* res = XRRGetScreenResourcesCurrent(dpy, DefaultRootWindow(dpy));
    for (int i = 0; res && i < res->noutput && !ok; ++i) {
        XRROutputInfo* oi = XRRGetOutputInfo(dpy, res, res->outputs[i]);
        if (oi && oi->connection == RR_Connected && oi->crtc) {
            const bool match = name.empty() ? std::strncmp(oi->name, "eDP", 3) == 0 : name == oi->name;
            if (match) {
                XRRCrtcInfo* ci = XRRGetCrtcInfo(dpy, res, oi->crtc);
                if (ci) {
                    out = {ci->x, ci->y, int(ci->width), int(ci->height)};
                    found_name = oi->name;
                    ok = true;
                    XRRFreeCrtcInfo(ci);
                }
            }
        }
        if (oi) XRRFreeOutputInfo(oi);
    }
    if (res) XRRFreeScreenResources(res);
    XCloseDisplay(dpy);
    return ok;
}

bool ScreenGrabber::start(const XRect& rect, int fps, int cursor_scale) {
    stop();
    rect_ = rect;
    fps_target_ = fps < 1 ? 1 : fps;
    cursor_scale_ = cursor_scale < 1 ? 1 : cursor_scale;
    ready_.assign(size_t(rect.w) * rect.h * 4, 0);
    fresh_ = false;
    quit_ = false;
    frames_ = 0;
    latest_ = {};
    const int n = 3;
    for (int i = 0; i < n; ++i) threads_.emplace_back(&ScreenGrabber::run, this, i, n);
    return true;
}

void ScreenGrabber::stop() {
    quit_ = true;
    for (auto& t : threads_) if (t.joinable()) t.join();
    threads_.clear();
}

bool ScreenGrabber::take(std::vector<uint8_t>& out) {
    std::lock_guard<std::mutex> lock(mu_);
    if (!fresh_) return false;
    out.swap(ready_);
    fresh_ = false;
    return true;
}

void ScreenGrabber::run(int index, int count) {
    using clock = std::chrono::steady_clock;
    Display* dpy = XOpenDisplay(nullptr);
    if (!dpy) { std::fprintf(stderr, "Capture: cannot open X display\n"); return; }
    if (!XShmQueryExtension(dpy)) { std::fprintf(stderr, "Capture: XShm not available\n"); XCloseDisplay(dpy); return; }
    int ev = 0, er = 0;
    const bool have_fixes = XFixesQueryExtension(dpy, &ev, &er);
    Window root = DefaultRootWindow(dpy);
    XWindowAttributes wa;
    XGetWindowAttributes(dpy, root, &wa);

    XShmSegmentInfo shm{};
    XImage* img = XShmCreateImage(dpy, wa.visual, unsigned(wa.depth), ZPixmap, nullptr, &shm, unsigned(rect_.w), unsigned(rect_.h));
    if (!img || img->bits_per_pixel != 32) { std::fprintf(stderr, "Capture: unsupported pixel format\n"); XCloseDisplay(dpy); return; }
    shm.shmid = shmget(IPC_PRIVATE, size_t(img->bytes_per_line) * img->height, IPC_CREAT | 0600);
    shm.shmaddr = img->data = static_cast<char*>(shmat(shm.shmid, nullptr, 0));
    shm.readOnly = False;
    XShmAttach(dpy, &shm);
    XSync(dpy, False);
    shmctl(shm.shmid, IPC_RMID, nullptr);  // freed automatically when we detach

    std::vector<uint8_t> back(size_t(rect_.w) * rect_.h * 4);
    const auto period = std::chrono::microseconds(1000000LL * count / fps_target_);
    auto next = clock::now() + period * index / count;
    auto stat_t = clock::now();
    int stat_n = 0;
    double acc_grab = 0, acc_copy = 0;
    long last_total = 0;
    while (!quit_) {
        next += period;
        back.resize(size_t(rect_.w) * rect_.h * 4);  // the buffer swapped back from the consumer may be empty
        const auto t_a = clock::now();
        const bool got = XShmGetImage(dpy, root, img, rect_.x, rect_.y, AllPlanes);
        const auto t_b = clock::now();
        acc_grab += std::chrono::duration<double, std::milli>(t_b - t_a).count();
        if (got) {
            if (img->bytes_per_line == rect_.w * 4) std::memcpy(back.data(), img->data, back.size());
            else for (int y = 0; y < rect_.h; ++y) std::memcpy(&back[size_t(y) * rect_.w * 4], img->data + size_t(y) * img->bytes_per_line, size_t(rect_.w) * 4);
            if (have_fixes) {
                if (XFixesCursorImage* c = XFixesGetCursorImage(dpy)) {
                    const int sc = cursor_scale_;
                    const int cx = c->x - rect_.x - int(c->xhot) * sc, cy = c->y - rect_.y - int(c->yhot) * sc;
                    for (int j = 0; j < int(c->height) * sc; ++j) {
                        const int dy = cy + j;
                        if (dy < 0 || dy >= rect_.h) continue;
                        for (int i = 0; i < int(c->width) * sc; ++i) {
                            const int dx = cx + i;
                            if (dx < 0 || dx >= rect_.w) continue;
                            const uint32_t p = uint32_t(c->pixels[size_t(j / sc) * c->width + size_t(i / sc)]);  // premultiplied ARGB
                            const uint32_t a = p >> 24;
                            if (!a) continue;
                            uint8_t* d = &back[(size_t(dy) * rect_.w + dx) * 4];
                            const uint32_t inv = 255 - a;
                            d[0] = uint8_t(std::min<uint32_t>(255, (p & 0xff) + d[0] * inv / 255));
                            d[1] = uint8_t(std::min<uint32_t>(255, ((p >> 8) & 0xff) + d[1] * inv / 255));
                            d[2] = uint8_t(std::min<uint32_t>(255, ((p >> 16) & 0xff) + d[2] * inv / 255));
                        }
                    }
                    XFree(c);
                }
            }
            {
                std::lock_guard<std::mutex> lock(mu_);
                if (t_a > latest_) { ready_.swap(back); latest_ = t_a; fresh_ = true; }   // an older frame from the other thread is dropped
            }
            ++frames_;
            ++stat_n;
            acc_copy += std::chrono::duration<double, std::milli>(clock::now() - t_b).count();
        }
        const auto now = clock::now();
        if (index == 0 && now - stat_t > std::chrono::seconds(2)) {
            const long total = frames_;
            fps_ = (total - last_total) / std::chrono::duration<double>(now - stat_t).count();
            last_total = total;
            if (stat_n) { grab_ms_ = acc_grab / stat_n; copy_ms_ = acc_copy / stat_n; }
            acc_grab = acc_copy = 0;
            stat_n = 0;
            stat_t = now;
        }
        if (next > now) std::this_thread::sleep_for(next - now);
        else next = now;  // can't keep up: don't accumulate debt
    }
    XShmDetach(dpy, &shm);
    img->data = nullptr;  // shared memory, not malloc'd
    XDestroyImage(img);
    shmdt(shm.shmaddr);
    XCloseDisplay(dpy);
}
