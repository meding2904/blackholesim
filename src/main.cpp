#include <algorithm>
#include <cerrno>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "application.hpp"
#include "simulation_parameters.hpp"

namespace {

constexpr int kMaxBenchFrames = 100000;

void printUsage() {
    std::printf(
        "Kerr black hole - real-time gravitational lensing\n"
        "\n"
        "Usage: blackhole [options]\n"
        "\n"
        "Options:\n"
        "  --size WxH         window size (default 1440x900)\n"
        "  --quality NAME     performance | balanced | high\n"
        "  --distance R       camera distance in r_g (default 45)\n"
        "  --yaw RAD          camera azimuth in radians (default 0.70)\n"
        "  --pitch RAD        camera elevation in radians (default 0.16)\n"
        "  --spin A           dimensionless spin a/M, 0..0.998 (default 0.6)\n"
        "  --rotation X       disk rotation speed multiplier (default 180)\n"
        "  --temperature K    disk temperature in kelvin (default 6800)\n"
        "  --time SECONDS     initial disk animation phase\n"
        "  --paused           start with the disk animation paused\n"
        "  --no-ui            hide the control panel\n"
        "  --capture FILE     render a few frames, save a PNG and exit\n"
        "  --frames N         frames to render before capturing (default 20)\n"
        "  --bench N          render N frames with vsync off, print frame times, exit\n"
        "  --help             show this text\n"
        "\n"
        "Controls:\n"
        "  left drag orbit   wheel zoom   middle drag elevation\n"
        "  space pause       R reset      H toggle UI       esc quit\n");
}

// Strict decimal parse: the entire string must be consumed and the result must
// be finite and representable as a float. NaN/inf and ERANGE are rejected.
bool parseFloat(const char* value, float& out) {
    if (value == nullptr || *value == '\0') return false;
    errno = 0;
    char* end = nullptr;
    const double parsed = std::strtod(value, &end);
    if (end == value || *end != '\0' || errno == ERANGE) return false;
    if (!std::isfinite(parsed)) return false;
    if (parsed < -FLT_MAX || parsed > FLT_MAX) return false;
    out = static_cast<float>(parsed);
    return true;
}

bool parseInteger(const char* value, long& out) {
    if (value == nullptr || *value == '\0') return false;
    errno = 0;
    char* end = nullptr;
    const long parsed = std::strtol(value, &end, 10);
    if (end == value || *end != '\0' || errno == ERANGE) return false;
    out = parsed;
    return true;
}

bool parseSize(const char* value, int& width, int& height) {
    if (value == nullptr) return false;

    errno = 0;
    char* end = nullptr;
    const unsigned long w = std::strtoul(value, &end, 10);
    if (end == value || errno == ERANGE || (*end != 'x' && *end != 'X')) return false;

    const char* heightText = end + 1;
    errno = 0;
    char* heightEnd = nullptr;
    const unsigned long h = std::strtoul(heightText, &heightEnd, 10);
    if (heightEnd == heightText || *heightEnd != '\0' || errno == ERANGE) return false;

    if (w < 320 || h < 240 || w > 7680 || h > 4320) return false;
    width = static_cast<int>(w);
    height = static_cast<int>(h);
    return true;
}

bool parseQuality(const char* value, int& quality) {
    if (std::strcmp(value, "performance") == 0 || std::strcmp(value, "perf") == 0 ||
        std::strcmp(value, "0") == 0) {
        quality = 0;
        return true;
    }
    if (std::strcmp(value, "balanced") == 0 || std::strcmp(value, "1") == 0) {
        quality = 1;
        return true;
    }
    if (std::strcmp(value, "high") == 0 || std::strcmp(value, "2") == 0) {
        quality = 2;
        return true;
    }
    return false;
}

}  // namespace

int main(int argc, char** argv) {
    bh::RunOptions options;

    const auto invalidValue = [](const char* name) {
        std::fprintf(stderr, "Invalid value for %s (try --help)\n", name);
        printUsage();
        return 1;
    };

    for (int i = 1; i < argc; ++i) {
        const char* arg = argv[i];
        const auto next = [&](const char* name) -> const char* {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "Missing value for %s\n", name);
                printUsage();
                std::exit(1);
            }
            return argv[++i];
        };

        if (std::strcmp(arg, "--help") == 0 || std::strcmp(arg, "-h") == 0) {
            printUsage();
            return 0;
        } else if (std::strcmp(arg, "--size") == 0) {
            if (!parseSize(next("--size"), options.width, options.height)) {
                return invalidValue("--size (expected WxH, e.g. 1600x900)");
            }
        } else if (std::strcmp(arg, "--quality") == 0) {
            if (!parseQuality(next("--quality"), options.quality)) {
                return invalidValue("--quality (performance|balanced|high)");
            }
        } else if (std::strcmp(arg, "--spin") == 0) {
            if (!parseFloat(next("--spin"), options.spin)) return invalidValue("--spin");
        } else if (std::strcmp(arg, "--distance") == 0) {
            if (!parseFloat(next("--distance"), options.distance)) {
                return invalidValue("--distance");
            }
        } else if (std::strcmp(arg, "--yaw") == 0) {
            if (!parseFloat(next("--yaw"), options.yaw)) return invalidValue("--yaw");
        } else if (std::strcmp(arg, "--pitch") == 0) {
            if (!parseFloat(next("--pitch"), options.pitch)) return invalidValue("--pitch");
        } else if (std::strcmp(arg, "--rotation") == 0) {
            if (!parseFloat(next("--rotation"), options.rotationSpeed)) {
                return invalidValue("--rotation");
            }
        } else if (std::strcmp(arg, "--temperature") == 0) {
            if (!parseFloat(next("--temperature"), options.temperature)) {
                return invalidValue("--temperature");
            }
        } else if (std::strcmp(arg, "--time") == 0) {
            if (!parseFloat(next("--time"), options.startTime)) return invalidValue("--time");
        } else if (std::strcmp(arg, "--bench") == 0) {
            long frames = 0;
            if (!parseInteger(next("--bench"), frames)) return invalidValue("--bench");
            if (frames < 1 || frames > kMaxBenchFrames) {
                std::fprintf(stderr, "Warning: --bench clamped to the range 1..%d\n",
                             kMaxBenchFrames);
            }
            options.benchFrames =
                static_cast<int>(std::clamp(frames, 1L, static_cast<long>(kMaxBenchFrames)));
        } else if (std::strcmp(arg, "--frames") == 0) {
            long frames = 0;
            if (!parseInteger(next("--frames"), frames)) return invalidValue("--frames");
            if (frames < 1 || frames > kMaxBenchFrames) {
                std::fprintf(stderr, "Warning: --frames clamped to the range 1..%d\n",
                             kMaxBenchFrames);
            }
            options.captureFrames =
                static_cast<int>(std::clamp(frames, 1L, static_cast<long>(kMaxBenchFrames)));
        } else if (std::strcmp(arg, "--capture") == 0) {
            options.capturePath = next("--capture");
        } else if (std::strcmp(arg, "--paused") == 0) {
            options.paused = true;
        } else if (std::strcmp(arg, "--no-ui") == 0) {
            options.noUi = true;
        } else {
            std::fprintf(stderr, "Unknown option: %s (try --help)\n", arg);
            printUsage();
            return 1;
        }
    }

    bh::Application app;
    return app.run(options);
}
