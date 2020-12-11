#include <ev3dev.h>
#include <vector>

int main() {
    auto allMotors = std::vector<ev3dev::motor>{
        {ev3dev::OUTPUT_A},
        {ev3dev::OUTPUT_B},
        {ev3dev::OUTPUT_C},
        {ev3dev::OUTPUT_D}
    };

    for (auto& mtr : allMotors) {
        if (mtr.connected()) {
            mtr.reset();
        }
    }
    
    return 0;
}