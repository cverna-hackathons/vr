// A synthetic desktop image (BGRA, w*h*4) with small text, used by `display --shot` instead of the real screen.
#pragma once
#include <cstdint>
#include <vector>
std::vector<uint8_t> make_test_desktop(int w, int h);
