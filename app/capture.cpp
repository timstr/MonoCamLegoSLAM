#include <Camera.hpp>
#include <LineSegmentDetector.hpp>
#include <VanishingPointEstimator.hpp>

#include <ev3dev.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

namespace {
    constexpr auto pi = 3.141592653589793238463;
} // anonymous namespace

int main(int argc, char **argv){
    // runTests();
    // return 0;

    auto cam = Camera{};
    auto img = Image{};

    auto mtrL = ev3dev::motor{ev3dev::OUTPUT_A};
    auto mtrR = ev3dev::motor{ev3dev::OUTPUT_D};

    mtrL.set_stop_action(ev3dev::motor::stop_action_brake);
    mtrR.set_stop_action(ev3dev::motor::stop_action_brake);

    mtrL.set_speed_sp(50);
    mtrR.set_speed_sp(-50);

    const auto camParams = CameraParameters{
        176,       // Width,  taken from output files
        144,       // Height, taken from output files
        pi * 0.25, // (45 degrees) horizontal FOV, manually measured
        pi * 0.2   // (36 degrees) vertical FOV, manually measured
    };

    for (int i = 0; i < 30; ++i) {

        std::cout << "Capturing...";
        std::cout.flush();
        cam.capture(img);
        std::cout << " done!\nSaving image...";
        std::cout.flush();

        save_ppm(img, "image " + std::to_string(i) + ".ppm");

        std::cout << " done!\nDetecting lines...";
        std::cout.flush();

        auto ls = detectLines(img, 15.0);

        std::cout << " done!\nRendering lines...";
        std::cout.flush();

        renderLines(ls, img, false);

        std::cout << " done!\nSaving lines...";
        std::cout.flush();
        
        save_ppm(img, "lines " + std::to_string(i) + ".ppm");

        std::cout << " done!\nEstimating VP...";
        std::cout.flush();

        auto gridDir = estimateVanishingPoint(ls, camParams, 0);

        std::cout << " done!\nGrid angle: " << std::to_string(gridDir) << "\nDrawing grid.";
        std::cout.flush();

        ls.clear();
        if (!std::isnan(gridDir)) {
            const auto r = 50.0;
            const auto a = r * std::cos(-gridDir);
            const auto b = r * std::sin(-gridDir);
            const auto cx = 88.0;
            const auto cy = 72.0;

            ls.push_back(LineSegment{
                Point2D{cx + b, cy + a},
                Point2D{cx - b, cy - a},
                1.0
            });

            ls.push_back(LineSegment{
                Point2D{cx + a, cy - b},
                Point2D{cx - a, cy + b},
                1.0
            });
        }
        std::cout << '.';
        renderLines(ls, img, true);
        std::cout << '.';
        save_ppm(img, "grid " + std::to_string(i) + ".ppm");

        std::cout << " done!\n";

        {
            mtrL.run_forever();
            mtrR.run_forever();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            mtrL.stop();
            mtrR.stop();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    return 0;
}
