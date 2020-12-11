#pragma once

#include <cstdint>
#include <string>
#include <vector>

/**
 * Class representing a greyscale image
 */
class Image {
public:
    Image();

    Image(unsigned int width, unsigned int height);

    void resize(unsigned int width, unsigned int height);

    std::uint8_t operator()(unsigned int x, unsigned int y) const noexcept;

    std::uint8_t& operator()(unsigned int x, unsigned int y) noexcept;

    unsigned int width() const noexcept;

    unsigned int height() const noexcept;

private:
    unsigned int m_width;
    unsigned int m_height;
    std::vector<std::uint8_t> m_data;
};

void save_ppm(const Image& img, const std::string& filename);
