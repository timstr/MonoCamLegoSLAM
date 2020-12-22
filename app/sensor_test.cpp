#include <chrono>
#include <iostream>
#include <thread>

#include <ev3dev.h>

using namespace ev3dev;

int main() {

    auto accel = sensor("ev3-ports:in1:i2c1", {"ht-nxt-accel"});

    if (!accel.connected()) {
        std::cout << "No sensor detected\n";
        return 1;
    }

    accel.set_mode("ALL");

    for (int i = 0; i < 500; ++i) {
        int x = accel.value(0);
        int y = accel.value(1);
        int z = accel.value(2);
        if (x > 127) {
            x -= 256;
        }
        if (y > 127) {
            y -= 256;
        }
        if (z > 127) {
            z -= 256;
        }
        x = (x << 2) + accel.value(3);
        y = (y << 2) + accel.value(4);
        z = (z << 2) + accel.value(5);
        std::cout << x << ' ' << y << ' ' << z << '\n';
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return 0;
}