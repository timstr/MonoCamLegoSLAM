#pragma once

#include <cstdint>
#include <functional>

enum class Key : std::uint8_t {
    W,
    A,
    S,
    D
};


/**
 * Freezes the command line and renders it completely useless.
 * On the plus side, it tells you in realtime when keypresses
 * happen via the callback function.
 */
void logKeys(std::function<void(Key)> callback);