#include <iostream>

#include <ev3dev.h>

int main() {
    const auto& bat = ev3dev::power_supply::battery;
    std::cout << bat.measured_volts() << " volts\n";
    std::cout << bat.measured_amps() << " amps\n";

    return 0;
}
