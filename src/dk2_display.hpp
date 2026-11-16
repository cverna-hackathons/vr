// Makes sure the DK2 output is set up before a program looks for it: runs tools/dk2-setup.sh, which does nothing when the
// headset display is already configured. Failure is not fatal (e.g. headset unplugged): the caller carries on and reports
// its own error if it can't find the display.
#pragma once
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

inline void ensure_dk2_display() {
    char buf[4096];
    const ssize_t n = readlink("/proc/self/exe", buf, sizeof buf - 1);
    std::string script;
    if (n > 0) {
        std::string exe(buf, size_t(n));
        const size_t slash = exe.rfind('/');
        if (slash != std::string::npos) script = exe.substr(0, slash) + "/../tools/dk2-setup.sh";  // build/<exe> -> tools/
    }
    if (script.empty() || access(script.c_str(), X_OK) != 0) script = "tools/dk2-setup.sh";  // fall back to the working directory
    if (access(script.c_str(), X_OK) != 0) return;

    std::string cmd = "'";
    for (char c : script) { if (c == '\'') cmd += "'\\''"; else cmd += c; }
    cmd += "' 2>&1";
    FILE* p = popen(cmd.c_str(), "r");
    if (!p) return;
    std::string out;
    char line[512];
    while (std::fgets(line, sizeof line, p)) out += line;
    const int rc = pclose(p);
    if (!out.empty()) std::fputs(out.c_str(), stdout);
    if (rc != 0) { std::printf("DK2 display setup did not succeed (is the headset connected over HDMI?). Continuing.\n"); return; }
    if (out.find("nothing to do") == std::string::npos)  // we just changed the layout: give X a moment to settle
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
}
