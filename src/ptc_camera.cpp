#include "ptc_camera.hpp"
#include <algorithm>
#include <cstdlib>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <initializer_list>
#include <linux/usb/video.h>
#include <linux/uvcvideo.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <unistd.h>

namespace {
int xioctl(int fd, unsigned long req, void* arg) {
    int r;
    do { r = ioctl(fd, req, arg); } while (r == -1 && errno == EINTR);
    return r;
}

// SET_CUR the request, then poll GET_CUR until the first reply byte equals `expect`.
bool xu_set_get(int fd, uint8_t sel, uint8_t* buf, uint16_t len, uint8_t expect) {
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
    return true;
}

constexpr uint8_t kSensorAddr = 0x4c << 1;

bool i2c_write(int fd, uint8_t reg, uint16_t val) {
    uint8_t b[6] = {0x04, kSensorAddr, reg, uint8_t(val >> 8), uint8_t(val & 255), 0};
    return xu_set_get(fd, 2, b, 6, 0x04) && b[0] == 0x04 && b[1] == kSensorAddr && b[2] == reg && b[5] == 0;
}

bool i2c_read(int fd, uint8_t reg, uint16_t* val) {
    uint8_t b[6] = {0x84, kSensorAddr, reg, 0, 0, 0};
    if (!xu_set_get(fd, 2, b, 6, 0x84) || b[0] != 0x84 || b[4] != 0 || b[5] != 0) return false;
    *val = uint16_t(b[1] << 8 | b[2]);
    return true;
}

enum : uint8_t { R_CHIP_ID = 0x00, R_HBLANK = 0x05, R_VBLANK = 0x06, R_CHIP_CTRL = 0x07, R_COARSE_SHUTTER = 0x0b,
                 R_LED_OUT = 0x1b, R_ANALOG_GAIN = 0x35, R_BLACK_CTRL = 0x47, R_BLACK_VAL = 0x48,
                 R_AEC_AGC = 0xaf, R_FINE_SHUTTER = 0xd5 };
constexpr uint16_t CC_MASTER = 1 << 3, CC_SNAPSHOT = 3 << 3, CC_DOUT = 1 << 7, CC_SEQ = 1 << 8;

void bridge_setup_unknown_3(int fd) {  // replays values captured from the Windows driver (purpose unknown)
    uint8_t a[3] = {0x80, 0x14, 0x00}, b[3] = {0xa0, 0xf0, 0x00};
    uvc_xu_control_query xu{};
    xu.unit = 4; xu.selector = 3; xu.size = 3;
    for (uint8_t* d : {a, b}) {
        xu.data = d;
        xu.query = UVC_SET_CUR;
        ioctl(fd, UVCIOC_CTRL_QUERY, &xu);
        xu.query = UVC_GET_CUR;
        ioctl(fd, UVCIOC_CTRL_QUERY, &xu);
    }
}

bool sensor_enable_sync(int fd) {
    bridge_setup_unknown_3(fd);
    uint16_t id = 0;
    if (!i2c_read(fd, R_CHIP_ID, &id) || id != 0x1324) {
        std::fprintf(stderr, "PTC: unexpected sensor id 0x%04x\n", id);
        return false;
    }
    return i2c_write(fd, R_AEC_AGC, 0) && i2c_write(fd, R_BLACK_CTRL, 1) && i2c_write(fd, R_BLACK_VAL, 0x81) &&
           i2c_write(fd, R_ANALOG_GAIN, 16) && i2c_write(fd, R_VBLANK, 5) && i2c_write(fd, R_HBLANK, 94) &&
           i2c_write(fd, R_CHIP_CTRL, CC_MASTER | CC_DOUT | CC_SEQ) && i2c_write(fd, R_COARSE_SHUTTER, 11) &&
           i2c_write(fd, R_FINE_SHUTTER, 111) &&
           i2c_write(fd, R_CHIP_CTRL, CC_SNAPSHOT | CC_DOUT | CC_SEQ) && i2c_write(fd, R_LED_OUT, 0x02);
}

void sensor_disable_sync(int fd) {
    i2c_write(fd, R_LED_OUT, 0);
    i2c_write(fd, R_AEC_AGC, 0);
    i2c_write(fd, R_BLACK_CTRL, 0x80);
    i2c_write(fd, R_ANALOG_GAIN, 0x20);
    i2c_write(fd, R_VBLANK, 57);
    i2c_write(fd, R_HBLANK, 94);
    i2c_write(fd, R_CHIP_CTRL, CC_MASTER | CC_DOUT | CC_SEQ);
    i2c_write(fd, R_COARSE_SHUTTER, 0xf0);
    i2c_write(fd, R_FINE_SHUTTER, 0);
}

std::string find_ptc_node() {
    DIR* d = opendir("/sys/class/video4linux");
    if (!d) return "";
    std::string best;
    while (dirent* e = readdir(d)) {
        if (std::strncmp(e->d_name, "video", 5)) continue;
        std::string base = std::string("/sys/class/video4linux/") + e->d_name;
        char name[64] = {0}, idx[8] = {0};
        if (FILE* f = std::fopen((base + "/name").c_str(), "r")) { if (!std::fgets(name, sizeof name, f)) name[0] = 0; std::fclose(f); }
        if (FILE* f = std::fopen((base + "/index").c_str(), "r")) { if (!std::fgets(idx, sizeof idx, f)) idx[0] = 0; std::fclose(f); }
        if (std::strstr(name, "Camera DK2") && std::atoi(idx) == 0) best = std::string("/dev/") + e->d_name;
    }
    closedir(d);
    return best;
}
}  // namespace

bool PtcCamera::open(const std::string& device) {
    std::string path = device.empty() ? find_ptc_node() : device;
    if (path.empty()) { std::fprintf(stderr, "PTC: camera not found (is its USB cable connected?)\n"); return false; }
    fd_ = ::open(path.c_str(), O_RDWR);
    if (fd_ < 0) { std::perror(path.c_str()); return false; }
    v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd_, VIDIOC_G_FMT, &fmt) < 0) { std::perror("PTC G_FMT"); return false; }
    if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_GREY) { std::fprintf(stderr, "PTC: unexpected pixel format\n"); return false; }
    width_ = fmt.fmt.pix.width;
    height_ = fmt.fmt.pix.height;
    return true;
}

bool PtcCamera::start(bool sync) {
    if (fd_ < 0 || streaming_) return false;
    v4l2_requestbuffers req{};
    req.count = 4; req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd_, VIDIOC_REQBUFS, &req) < 0) { std::perror("PTC REQBUFS"); return false; }
    bufs_.resize(req.count);
    for (unsigned i = 0; i < req.count; ++i) {
        v4l2_buffer b{};
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory = V4L2_MEMORY_MMAP; b.index = i;
        xioctl(fd_, VIDIOC_QUERYBUF, &b);
        bufs_[i] = {mmap(nullptr, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, b.m.offset), b.length};
        xioctl(fd_, VIDIOC_QBUF, &b);
    }
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd_, VIDIOC_STREAMON, &type) < 0) { std::perror("PTC STREAMON"); return false; }
    streaming_ = true;
    // The bridge only answers I2C requests while streaming.
    if (sync) {
        synced_ = sensor_enable_sync(fd_);
        if (!synced_) std::fprintf(stderr, "PTC: could not enable sync exposure\n");
        return synced_;
    }
    return true;
}

bool PtcCamera::grab(PtcFrame& out, int timeout_ms) {
    if (!streaming_) return false;
    fd_set fds; FD_ZERO(&fds); FD_SET(fd_, &fds);
    timeval tv{timeout_ms / 1000, (timeout_ms % 1000) * 1000};
    if (select(fd_ + 1, &fds, nullptr, nullptr, &tv) <= 0) return false;
    v4l2_buffer b{};
    b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd_, VIDIOC_DQBUF, &b) < 0) return false;
    out.width = width_;
    out.height = height_;
    out.sequence = b.sequence;
    out.timestamp_us = uint64_t(b.timestamp.tv_sec) * 1000000ull + uint64_t(b.timestamp.tv_usec);
    const uint8_t* src = static_cast<uint8_t*>(bufs_[b.index].p);
    out.pix.assign(src, src + std::min<size_t>(b.bytesused, size_t(width_) * height_));
    out.pix.resize(size_t(width_) * height_);
    xioctl(fd_, VIDIOC_QBUF, &b);
    return true;
}

PtcIntrinsics PtcCamera::read_intrinsics() {
    PtcIntrinsics out;
    if (fd_ < 0) return out;
    uint8_t data[128];
    for (int off = 0; off < 128; off += 32) {
        uint8_t b[59] = {0x82, 0xa0, uint8_t((0x2000 + off) >> 8), uint8_t((0x2000 + off) & 255), 32};
        if (!xu_set_get(fd_, 5, b, sizeof b, 0x82) || b[0] != 0x82 || b[1] != 32) return out;
        std::memcpy(data + off, b + 2, 32);
    }
    auto d = [&](int o) { double v; std::memcpy(&v, data + o, 8); return v; };
    out.fx = d(18); out.fy = d(30); out.cx = d(42); out.cy = d(54);
    out.k1 = d(66); out.k2 = d(78); out.p1 = d(90); out.p2 = d(102); out.k3 = d(114);
    out.valid = out.fx > 100 && out.fx < 2000 && out.fy > 100 && out.fy < 2000;
    return out;
}

void PtcCamera::stop() {
    if (fd_ < 0) return;
    if (synced_) { sensor_disable_sync(fd_); synced_ = false; }
    if (streaming_) {
        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        xioctl(fd_, VIDIOC_STREAMOFF, &type);
        streaming_ = false;
    }
    for (auto& b : bufs_) if (b.p) munmap(b.p, b.n);
    bufs_.clear();
    ::close(fd_);
    fd_ = -1;
}
