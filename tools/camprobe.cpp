// Probe the DK2 positional-tracking camera through V4L2 (no extra dependencies).
// Lists formats/sizes, captures a few frames, prints brightness stats and saves the last frame as a PGM.
//
// Build: g++ -O2 -o build/camprobe tools/camprobe.cpp
// Run:   build/camprobe [/dev/video0] [out.pgm] [--sync] [ctrl_id=value ...]
//   --sync  switch the MT9V034 sensor to snapshot mode, exposure triggered by the headset's sync cable
//           (short exposure: only the IR LEDs should show). Restored to free-running mode on exit.
//           Register sequence follows the open-source ouvrt driver (esp570.c / mt9v034.c).
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <initializer_list>
#include <fcntl.h>
#include <linux/usb/video.h>
#include <linux/uvcvideo.h>
#include <linux/videodev2.h>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <unistd.h>
#include <vector>

static int xioctl(int fd, unsigned long req, void* arg) {
    int r;
    do { r = ioctl(fd, req, arg); } while (r == -1 && errno == EINTR);
    return r;
}

static std::string fourcc(uint32_t f) {
    char s[5] = {char(f & 255), char(f >> 8 & 255), char(f >> 16 & 255), char(f >> 24 & 255), 0};
    return s;
}

// --- ESP570 bridge: MT9V034 sensor registers over I2C via UVC extension unit 4, selector 2 ---
// SET_CUR the request, then poll GET_CUR until the reply's first byte equals `expect` (the bridge may need a
// moment to complete the I2C transaction; other values, e.g. 0x8c, were seen while it is busy).
static bool xu_set_get(int fd, uint8_t sel, uint8_t* buf, uint16_t len, uint8_t expect) {
    uvc_xu_control_query xu{};
    xu.unit = 4; xu.selector = sel; xu.size = len; xu.data = buf;
    xu.query = UVC_SET_CUR;
    if (ioctl(fd, UVCIOC_CTRL_QUERY, &xu) < 0) return false;
    for (int attempt = 0; attempt < 50; ++attempt) {
        usleep(2000);
        xu.query = UVC_GET_CUR;
        if (ioctl(fd, UVCIOC_CTRL_QUERY, &xu) < 0) return false;
        if (buf[0] == expect) return true;
    }
    return true;  // caller inspects the reply
}

static constexpr uint8_t kSensorAddr = 0x4c << 1;

static bool i2c_write(int fd, uint8_t reg, uint16_t val) {
    uint8_t b[6] = {0x04, kSensorAddr, reg, uint8_t(val >> 8), uint8_t(val & 255), 0};
    if (!xu_set_get(fd, 2, b, 6, 0x04)) return false;
    return b[0] == 0x04 && b[1] == kSensorAddr && b[2] == reg && b[5] == 0;
}

static bool i2c_read(int fd, uint8_t reg, uint16_t* val) {
    uint8_t b[6] = {0x84, kSensorAddr, reg, 0, 0, 0};
    if (!xu_set_get(fd, 2, b, 6, 0x84)) { std::printf("  XU I2C read: ioctl failed: %s\n", std::strerror(errno)); return false; }
    if (b[0] != 0x84 || b[4] != 0 || b[5] != 0) {
        std::printf("  XU I2C read: unexpected reply %02x %02x %02x %02x %02x %02x\n", b[0], b[1], b[2], b[3], b[4], b[5]);
        return false;
    }
    *val = uint16_t(b[1] << 8 | b[2]);
    return true;
}

// MT9V034 registers / bits used below
enum : uint8_t { R_CHIP_ID = 0x00, R_HBLANK = 0x05, R_VBLANK = 0x06, R_CHIP_CTRL = 0x07, R_COARSE_SHUTTER = 0x0b,
                 R_LED_OUT = 0x1b, R_ANALOG_GAIN = 0x35, R_BLACK_CTRL = 0x47, R_BLACK_VAL = 0x48,
                 R_AEC_AGC = 0xaf, R_FINE_SHUTTER = 0xd5 };
constexpr uint16_t CC_MASTER = 1 << 3, CC_SNAPSHOT = 3 << 3, CC_DOUT = 1 << 7, CC_SEQ = 1 << 8;

// Two selector-3 writes the ouvrt driver replays from a Windows driver capture; purpose not documented.
static void bridge_setup_unknown_3(int fd) {
    uint8_t a[3] = {0x80, 0x14, 0x00}, b[3] = {0xa0, 0xf0, 0x00};
    uvc_xu_control_query xu{};
    xu.unit = 4; xu.selector = 3; xu.size = 3;
    for (uint8_t* d : {a, b}) {
        xu.data = d;
        xu.query = UVC_SET_CUR;
        int r1 = ioctl(fd, UVCIOC_CTRL_QUERY, &xu);
        xu.query = UVC_GET_CUR;
        int r2 = ioctl(fd, UVCIOC_CTRL_QUERY, &xu);
        std::printf("  bridge setup (sel 3): set=%d get=%d reply=%02x%02x%02x\n", r1, r2, d[0], d[1], d[2]);
    }
}

static bool sensor_enable_sync(int fd) {
    bridge_setup_unknown_3(fd);
    uint16_t id = 0;
    if (!i2c_read(fd, R_CHIP_ID, &id)) { std::printf("Sensor: I2C read failed\n"); return false; }
    std::printf("Sensor chip id: 0x%04x (expect 0x1324)\n", id);
    if (id != 0x1324) return false;
    bool ok = i2c_write(fd, R_AEC_AGC, 0) && i2c_write(fd, R_BLACK_CTRL, 1) && i2c_write(fd, R_BLACK_VAL, 0x81) &&
              i2c_write(fd, R_ANALOG_GAIN, 16) && i2c_write(fd, R_VBLANK, 5) && i2c_write(fd, R_HBLANK, 94) &&
              i2c_write(fd, R_CHIP_CTRL, CC_MASTER | CC_DOUT | CC_SEQ) && i2c_write(fd, R_COARSE_SHUTTER, 11) &&
              i2c_write(fd, R_FINE_SHUTTER, 111) &&
              i2c_write(fd, R_CHIP_CTRL, CC_SNAPSHOT | CC_DOUT | CC_SEQ) &&  // exposure now triggered by the HMD
              i2c_write(fd, R_LED_OUT, 0x02);
    std::printf("Sensor sync mode: %s\n", ok ? "enabled" : "FAILED");
    return ok;
}

static void sensor_disable_sync(int fd) {
    i2c_write(fd, R_LED_OUT, 0);
    i2c_write(fd, R_AEC_AGC, 0);
    i2c_write(fd, R_BLACK_CTRL, 0x80);
    i2c_write(fd, R_ANALOG_GAIN, 0x20);
    i2c_write(fd, R_VBLANK, 57);
    i2c_write(fd, R_HBLANK, 94);
    i2c_write(fd, R_CHIP_CTRL, CC_MASTER | CC_DOUT | CC_SEQ);
    i2c_write(fd, R_COARSE_SHUTTER, 0xf0);
    i2c_write(fd, R_FINE_SHUTTER, 0);
    std::printf("Sensor restored to free-running mode\n");
}

int main(int argc, char** argv) {
    const char* dev = argc > 1 ? argv[1] : "/dev/video0";
    const char* out = argc > 2 ? argv[2] : "frame.pgm";

    bool want_sync = false;
    for (int i = 1; i < argc; ++i) if (!std::strcmp(argv[i], "--sync")) want_sync = true;

    int fd = open(dev, O_RDWR);
    if (fd < 0) { std::perror(dev); return 1; }

    v4l2_capability cap{};
    xioctl(fd, VIDIOC_QUERYCAP, &cap);
    std::printf("Device: %s (%s), capture=%d streaming=%d\n", cap.card, cap.driver,
                !!(cap.device_caps & V4L2_CAP_VIDEO_CAPTURE), !!(cap.device_caps & V4L2_CAP_STREAMING));

    for (v4l2_fmtdesc f{}; (f.type = V4L2_BUF_TYPE_VIDEO_CAPTURE, xioctl(fd, VIDIOC_ENUM_FMT, &f) == 0); ++f.index) {
        std::printf("Format %u: %s '%s'\n", f.index, f.description, fourcc(f.pixelformat).c_str());
        for (v4l2_frmsizeenum s{}; (s.pixel_format = f.pixelformat, xioctl(fd, VIDIOC_ENUM_FRAMESIZES, &s) == 0); ++s.index) {
            if (s.type != V4L2_FRMSIZE_TYPE_DISCRETE) break;
            std::printf("   %ux%u", s.discrete.width, s.discrete.height);
            for (v4l2_frmivalenum iv{}; (iv.pixel_format = f.pixelformat, iv.width = s.discrete.width,
                                         iv.height = s.discrete.height, xioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &iv) == 0); ++iv.index)
                if (iv.type == V4L2_FRMIVAL_TYPE_DISCRETE)
                    std::printf(" %.0ffps", double(iv.discrete.denominator) / iv.discrete.numerator);
            std::printf("\n");
        }
    }

    // Camera controls: list them, then apply any id=value pairs given on the command line.
    for (v4l2_queryctrl q{}; (q.id |= V4L2_CTRL_FLAG_NEXT_CTRL, xioctl(fd, VIDIOC_QUERYCTRL, &q) == 0); ) {
        if (!(q.flags & V4L2_CTRL_FLAG_DISABLED)) {
            v4l2_control c{q.id, 0};
            xioctl(fd, VIDIOC_G_CTRL, &c);
            std::printf("Control 0x%08x %-28s min=%d max=%d step=%d default=%d current=%d\n", q.id, q.name,
                        q.minimum, q.maximum, q.step, q.default_value, c.value);
        }
    }
    for (int i = 3; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--sync")) continue;
        unsigned id; int val;
        if (std::sscanf(argv[i], "%i=%i", &id, &val) == 2) {
            v4l2_control c{id, val};
            std::printf("Set 0x%08x = %d: %s\n", id, val, xioctl(fd, VIDIOC_S_CTRL, &c) == 0 ? "ok" : std::strerror(errno));
        }
    }

    v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd, VIDIOC_G_FMT, &fmt) < 0) { std::perror("G_FMT"); return 1; }
    std::printf("Current: %ux%u '%s' sizeimage=%u bytesperline=%u\n", fmt.fmt.pix.width, fmt.fmt.pix.height,
                fourcc(fmt.fmt.pix.pixelformat).c_str(), fmt.fmt.pix.sizeimage, fmt.fmt.pix.bytesperline);

    v4l2_requestbuffers req{};
    req.count = 4; req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd, VIDIOC_REQBUFS, &req) < 0) { std::perror("REQBUFS"); return 1; }
    struct Buf { void* p; size_t n; };
    std::vector<Buf> bufs(req.count);
    for (unsigned i = 0; i < req.count; ++i) {
        v4l2_buffer b{};
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory = V4L2_MEMORY_MMAP; b.index = i;
        xioctl(fd, VIDIOC_QUERYBUF, &b);
        bufs[i] = {mmap(nullptr, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, b.m.offset), b.length};
        xioctl(fd, VIDIOC_QBUF, &b);
    }
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd, VIDIOC_STREAMON, &type) < 0) { std::perror("STREAMON"); return 1; }
    // The ESP570 bridge only answers I2C requests while streaming (same order as the ouvrt driver).
    bool synced = want_sync && sensor_enable_sync(fd);

    const int kFrames = 30;
    int got = 0;
    std::vector<uint8_t> last;
    for (int i = 0; i < kFrames; ++i) {
        fd_set fds; FD_ZERO(&fds); FD_SET(fd, &fds);
        timeval tv{2, 0};
        if (select(fd + 1, &fds, nullptr, nullptr, &tv) <= 0) { std::printf("Timeout waiting for frame %d\n", i); break; }
        v4l2_buffer b{};
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory = V4L2_MEMORY_MMAP;
        if (xioctl(fd, VIDIOC_DQBUF, &b) < 0) { std::perror("DQBUF"); break; }
        last.assign(static_cast<uint8_t*>(bufs[b.index].p), static_cast<uint8_t*>(bufs[b.index].p) + b.bytesused);
        ++got;
        if (i % 10 == 0) {
            unsigned mx = 0, bright = 0; uint64_t sum = 0;
            for (uint8_t v : last) { sum += v; if (v > mx) mx = v; if (v > 200) ++bright; }
            std::printf("frame %2d: %zu bytes, mean %.1f, max %u, pixels>200: %u\n", i, last.size(),
                        double(sum) / last.size(), mx, bright);
        }
        xioctl(fd, VIDIOC_QBUF, &b);
    }
    if (synced) sensor_disable_sync(fd);
    xioctl(fd, VIDIOC_STREAMOFF, &type);
    std::printf("Captured %d/%d frames\n", got, kFrames);

    // Save as greyscale PGM. For YUYV take Y (every 2nd byte); for GREY/Y8 use bytes directly.
    if (!last.empty()) {
        const unsigned w = fmt.fmt.pix.width, h = fmt.fmt.pix.height;
        const bool yuyv = fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV;
        std::vector<uint8_t> grey(w * h, 0);
        for (unsigned y = 0; y < h; ++y)
            for (unsigned x = 0; x < w; ++x) {
                size_t src = yuyv ? (size_t(y) * fmt.fmt.pix.bytesperline + x * 2) : (size_t(y) * fmt.fmt.pix.bytesperline + x);
                if (src < last.size()) grey[y * w + x] = last[src];
            }
        if (FILE* f = std::fopen(out, "wb")) {
            std::fprintf(f, "P5\n%u %u\n255\n", w, h);
            std::fwrite(grey.data(), 1, grey.size(), f);
            std::fclose(f);
            std::printf("Saved %s\n", out);
        }
    }
    close(fd);
    return got > 0 ? 0 : 2;
}
