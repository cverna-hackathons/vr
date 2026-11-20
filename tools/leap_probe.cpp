// Leap Motion probe: lists the formats of the controller's video node and saves a few frames as PGM.
// Usage: leap_probe [/dev/videoN] [OUTPREFIX]
#include <cerrno>
#include <cstdlib>
#include <sys/select.h>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <vector>

static int xioctl(int fd, unsigned long r, void* a) { int x; do { x = ioctl(fd, r, a); } while (x == -1 && errno == EINTR); return x; }

int main(int argc, char** argv) {
    setvbuf(stdout, nullptr, _IONBF, 0);
    const char* dev = argc > 1 ? argv[1] : "/dev/video4";
    const std::string out = argc > 2 ? argv[2] : "leap";
    int fd = open(dev, O_RDWR);
    if (fd < 0) { std::perror(dev); return 1; }
    v4l2_capability cap{};
    xioctl(fd, VIDIOC_QUERYCAP, &cap);
    std::printf("%s: %s caps=0x%x\n", dev, cap.card, cap.device_caps);
    for (int i = 0;; ++i) {
        v4l2_fmtdesc fd_{}; fd_.index = i; fd_.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (xioctl(fd, VIDIOC_ENUM_FMT, &fd_) < 0) break;
        char fcc[5] = {char(fd_.pixelformat), char(fd_.pixelformat >> 8), char(fd_.pixelformat >> 16), char(fd_.pixelformat >> 24), 0};
        std::printf("format %s (%s)\n", fcc, fd_.description);
        for (int j = 0;; ++j) {
            v4l2_frmsizeenum s{}; s.index = j; s.pixel_format = fd_.pixelformat;
            if (xioctl(fd, VIDIOC_ENUM_FRAMESIZES, &s) < 0) break;
            if (s.type == V4L2_FRMSIZE_TYPE_DISCRETE) std::printf("  %ux%u\n", s.discrete.width, s.discrete.height);
        }
    }
    v4l2_format f{}; f.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(fd, VIDIOC_G_FMT, &f);
    if (argc > 4) { f.fmt.pix.width = unsigned(atoi(argv[3])); f.fmt.pix.height = unsigned(atoi(argv[4])); f.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV; f.fmt.pix.field = V4L2_FIELD_NONE;
        if (xioctl(fd, VIDIOC_S_FMT, &f) < 0) std::perror("S_FMT"); xioctl(fd, VIDIOC_G_FMT, &f); }
    std::printf("current: %ux%u sizeimage %u\n", f.fmt.pix.width, f.fmt.pix.height, f.fmt.pix.sizeimage);
    if (const char* c = getenv("CTRLS")) {   // CTRLS="0x980900=15,0x9a0902=100": set V4L2 controls before streaming
        std::string list = c; size_t pos = 0;
        while (pos < list.size()) {
            size_t e = list.find(',', pos); if (e == std::string::npos) e = list.size();
            unsigned id = 0; int val = 0;
            if (std::sscanf(list.substr(pos, e - pos).c_str(), "%i=%i", &id, &val) == 2) {
                v4l2_control ctl{id, val};
                std::printf("set %#x = %d: %s\n", id, val, xioctl(fd, VIDIOC_S_CTRL, &ctl) < 0 ? std::strerror(errno) : "ok");
            }
            pos = e + 1;
        }
    }
    v4l2_requestbuffers rq{}; rq.count = 4; rq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; rq.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd, VIDIOC_REQBUFS, &rq) < 0) { std::perror("REQBUFS"); return 1; }
    std::vector<std::pair<void*, size_t>> b(rq.count);
    for (unsigned i = 0; i < rq.count; ++i) {
        v4l2_buffer q{}; q.type = rq.type; q.memory = V4L2_MEMORY_MMAP; q.index = i;
        xioctl(fd, VIDIOC_QUERYBUF, &q);
        b[i] = {mmap(nullptr, q.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, q.m.offset), q.length};
        xioctl(fd, VIDIOC_QBUF, &q);
    }
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd, VIDIOC_STREAMON, &type) < 0) { std::perror("STREAMON"); return 1; }
    for (int n = 0; n < 40; ++n) {
        fd_set fs; FD_ZERO(&fs); FD_SET(fd, &fs); timeval tv{3, 0};
        if (select(fd + 1, &fs, nullptr, nullptr, &tv) <= 0) { std::printf("timeout waiting for frame %d\n", n); break; }
        v4l2_buffer q{}; q.type = rq.type; q.memory = V4L2_MEMORY_MMAP;
        if (xioctl(fd, VIDIOC_DQBUF, &q) < 0) { std::perror("DQBUF"); break; }
        if (n % 13 == 0 || n == 12) {
            const std::string name = out + std::to_string(n) + ".raw";
            if (FILE* o = std::fopen(name.c_str(), "wb")) { std::fwrite(b[q.index].first, 1, q.bytesused, o); std::fclose(o); }
            const unsigned char* p = static_cast<unsigned char*>(b[q.index].first);
            long sum = 0; for (unsigned i = 0; i < q.bytesused; ++i) sum += p[i];
            std::printf("frame %d bytes %u mean %.1f\n", n, q.bytesused, double(sum) / q.bytesused);
        }
        xioctl(fd, VIDIOC_QBUF, &q);
    }
    xioctl(fd, VIDIOC_STREAMOFF, &type);
}
