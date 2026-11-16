// Print the PTC lens calibration stored in the camera's EEPROM.
#include <cstdio>
#include "../src/ptc_camera.hpp"
int main() {
    PtcCamera cam;
    if (!cam.open()) return 1;
    PtcIntrinsics k = cam.read_intrinsics();
    if (!k.valid) {  // some firmware only answers while streaming
        cam.start(false);
        k = cam.read_intrinsics();
    }
    std::printf("valid=%d\nfx=%.4f fy=%.4f cx=%.4f cy=%.4f\nk1=%.6f k2=%.6f p1=%.6f p2=%.6f k3=%.6f\n",
                k.valid, k.fx, k.fy, k.cx, k.cy, k.k1, k.k2, k.p1, k.p2, k.k3);
    return k.valid ? 0 : 2;
}
