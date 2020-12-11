#include <Image.hpp>

#include <cassert>
#include <fstream>

Image::Image()
    : m_width{0}
    , m_height{0}
    , m_data{} {

}

Image::Image(unsigned int width, unsigned int height)
    : m_width(width)
    , m_height(height)
    , m_data(width * height, std::uint8_t{0}) {
    
}

void Image::resize(unsigned int width, unsigned int height) {
    if (width == m_width && height == m_height) {
        return;
    }
    m_data = std::vector<std::uint8_t>(width * height, std::uint8_t{0});
    m_width = width;
    m_height = height;
}

std::uint8_t Image::operator()(unsigned int x, unsigned int y) const noexcept {
    assert(x < m_width);
    assert(y < m_height);
    return m_data[y * m_width + x];
}

std::uint8_t& Image::operator()(unsigned int x, unsigned int y) noexcept {
    assert(x < m_width);
    assert(y < m_height);
    return m_data[y * m_width + x];
}

unsigned int Image::width() const noexcept {
    return m_width;
}

unsigned int Image::height() const noexcept {
    return m_height;
}



void save_ppm(const Image& img, const std::string& filename) {
    auto f = std::ofstream(filename, std::ios::out | std::ios::binary);

    f << "P6\n" << img.width() << ' ' << img.height() << "\n255\n";
    
    for (unsigned int y = 0; y < img.height(); ++y) {
        for (unsigned int x = 0; x < img.width(); ++x) {
            std::uint8_t p = img(x, y);
            const auto pp = reinterpret_cast<const char*>(&p);
            f.write(pp, 1);
            f.write(pp, 1);
            f.write(pp, 1);
        }
    }
}
