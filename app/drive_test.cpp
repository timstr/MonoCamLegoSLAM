#include <atomic>
#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>

#include <Camera.hpp>
#include <KeyLogger.hpp>

#include <ev3dev.h>

std::int64_t program_time_ms() {
    static auto clock = std::chrono::steady_clock{};
    static auto firstTime = clock.now();
    auto elapsed = clock.now() - firstTime;
    auto secsElapsed = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);
    auto nSecsElapsed = secsElapsed.count();
    return nSecsElapsed;
}

int main(){
    auto cam = Camera{};
    auto img = Image{};
    
    auto mtrL = ev3dev::motor{ev3dev::OUTPUT_A};
    auto mtrR = ev3dev::motor{ev3dev::OUTPUT_D};

    mtrL.reset();
    mtrR.reset();
    mtrL.set_stop_action(ev3dev::motor::stop_action_brake);
    mtrR.set_stop_action(ev3dev::motor::stop_action_brake);
    mtrL.set_speed_sp(0);
    mtrL.set_speed_sp(0);
    
    auto forwardSpeedLevel = 0;
    auto rotationSpeedLevel = 0;
    const auto maxLevel = 2;
    const auto speedPerLevel = 50;

    std::atomic<bool> done{false};

    auto logFile = std::ofstream{"motorLog.txt"};

    auto motorLogger = std::thread{[&]{
        while (!done.load()) {
            // NOTE: position apparently uses the full range of a signed 32-bit integer, so wrap-around shouldn't be an issue
            logFile << program_time_ms() << ' ' << mtrL.position() << ' ' << mtrR.position() << '\n';
            std::this_thread::sleep_for(std::chrono::milliseconds{100});
        }
    }};

    auto cameraLogger = std::thread{[&]{
        while (!done.load()) {
            cam.capture(img); // HACK: capturing twice to purge the otherwise old image and because V4L2 is complicated
            cam.capture(img);
            save_ppm(img, "img/img_" + std::to_string(program_time_ms()) + ".ppm");
            std::this_thread::sleep_for(std::chrono::milliseconds{1000});
        }
    }};

    logKeys([&](Key k){
        switch (k) {
        case Key::W:
            forwardSpeedLevel += 1;
            break;
        case Key::A:
            rotationSpeedLevel -= 1;
            break;
        case Key::S:
            forwardSpeedLevel -= 1;
            break;
        case Key::D:
            rotationSpeedLevel += 1;
            break;
        default:
            std::cout << "???\n";
            break;
        }
        forwardSpeedLevel = std::min(std::max(forwardSpeedLevel, -maxLevel), maxLevel);
        rotationSpeedLevel = std::min(std::max(rotationSpeedLevel, -maxLevel), maxLevel);

        const auto lSpeed = speedPerLevel * (forwardSpeedLevel + rotationSpeedLevel);
        const auto rSpeed = speedPerLevel * (forwardSpeedLevel - rotationSpeedLevel);

        if (lSpeed == 0) {
            mtrL.stop();
        } else {
            mtrL.set_speed_sp(lSpeed);
            mtrL.run_forever();
        }

        if (rSpeed == 0) {
            mtrR.stop();
        } else {
            mtrR.set_speed_sp(rSpeed);
            mtrR.run_forever();
        }

        // std::this_thread::yield();
    });

    done.store(true);

    mtrL.stop();
    mtrR.stop();

    motorLogger.join();
    cameraLogger.join();

    return 0;
}