#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "application.hpp"
#include "simulation_parameters.hpp"

namespace {

void printUsage() {
    std::printf(
        "Schwarzschild black hole - real-time gravitational lensing\n"
        "\n"
        "Usage: blackhole [options]\n"
        "\n"
        "Options:\n"
        "  --size WxH         window size (default 1440x900)\n"
        "  --quality NAME     performance | balanced | high\n"
        "  --distance R       camera distance in r_g (default 45)\n"
        "  --yaw RAD          camera azimuth in radians (default 0.70)\n"
        "  --pitch RAD        camera elevation in radians (default 0.16)\n"
        "  --time SECONDS     initial disk animation phase\n"
        "  --paused           start with the disk animation paused\n"
        "  --no-ui            hide the control panel\n"
        "  --capture FILE     render a few frames, save a PNG and exit\n"
        "  --frames N         frames to render before capturing (default 20)\n"
        "  --help             show this text\n"
        "\n"
        "Controls:\n"
        "  left drag orbit   wheel zoom   middle drag elevation\n"
        "  space pause       R reset      H toggle UI       esc quit\n");
}

bool parseSize(const char* value, int& width, int& height) {
    int w = 0;
    int h = 0;
    if (std::sscanf(value, "%dx%d", &w, &h) != 2) return false;
    if (w < 320 || h < 240 || w > 7680 || h > 4320) return false;
    width = w;
    height = h;
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

    for (int i = 1; i < argc; ++i) {
        const char* arg = argv[i];
        const auto next = [&](const char* name) -> const char* {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "Missing value for %s\n", name);
                std::exit(2);
            }
            return argv[++i];
        };

        if (std::strcmp(arg, "--help") == 0 || std::strcmp(arg, "-h") == 0) {
            printUsage();
            return 0;
        } else if (std::strcmp(arg, "--size") == 0) {
            if (!parseSize(next("--size"), options.width, options.height)) {
                std::fprintf(stderr, "Invalid --size, expected WxH (e.g. 1600x900)\n");
                return 2;
            }
        } else if (std::strcmp(arg, "--quality") == 0) {
            if (!parseQuality(next("--quality"), options.quality)) {
                std::fprintf(stderr, "Invalid --quality (performance|balanced|high)\n");
                return 2;
            }
        } else if (std::strcmp(arg, "--distance") == 0) {
            options.distance = static_cast<float>(std::atof(next("--distance")));
        } else if (std::strcmp(arg, "--yaw") == 0) {
            options.yaw = static_cast<float>(std::atof(next("--yaw")));
        } else if (std::strcmp(arg, "--pitch") == 0) {
            options.pitch = static_cast<float>(std::atof(next("--pitch")));
        } else if (std::strcmp(arg, "--time") == 0) {
            options.startTime = static_cast<float>(std::atof(next("--time")));
        } else if (std::strcmp(arg, "--frames") == 0) {
            options.captureFrames = std::atoi(next("--frames"));
        } else if (std::strcmp(arg, "--capture") == 0) {
            options.capturePath = next("--capture");
        } else if (std::strcmp(arg, "--paused") == 0) {
            options.paused = true;
        } else if (std::strcmp(arg, "--no-ui") == 0) {
            options.noUi = true;
        } else {
            std::fprintf(stderr, "Unknown option: %s (try --help)\n", arg);
            return 2;
        }
    }

    bh::Application app;
    return app.run(options);
}
