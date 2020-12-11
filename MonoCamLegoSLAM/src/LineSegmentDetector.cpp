#include <LineSegmentDetector.hpp>

#include <lsd.hpp>

#include <cmath>

Point2D::Point2D(double _x, double _y) noexcept
    : x(_x)
    , y(_y) {
    
}

LineSegment::LineSegment(Point2D _p0, Point2D _p1, double _width) noexcept
    : p0(_p0)
    , p1(_p1)
    , width(_width) {

}

std::vector<LineSegment> detectLines(const Image& img, double minLength) {
    std::vector<double> pixels;
    pixels.reserve(img.width() * img.height());
    for (unsigned int y = 0; y < img.height(); ++y) {
        for (unsigned int x = 0; x < img.width(); ++x) {
            pixels.push_back(static_cast<double>(img(x, y)));
        }
    }

    auto num_segments = int{};
    auto segments_raw = lsd(&num_segments, pixels.data(), img.width(), img.height());

    std::vector<LineSegment> out;

    for (int i = 0; i < num_segments; ++i) {
        auto seg_ptr = segments_raw + (7 * i);

        auto x1 = seg_ptr[0];
        auto y1 = seg_ptr[1];
        auto x2 = seg_ptr[2];
        auto y2 = seg_ptr[3];
        auto w  = seg_ptr[4];
        // auto p = seg_ptr[5];
        // auto nlog10nfa = seg_ptr[6];

        if (std::hypot(x2 - x1, y2 - y1) < minLength) {
            continue;
        }

        out.push_back(LineSegment{
            Point2D{x1, y1},
            Point2D{x2, y2},
            w
        });
    }

    return out;
}

void renderLines(const std::vector<LineSegment>& lines, Image& img, bool eraseImage) {
    if (eraseImage) {
        for (unsigned int y = 0; y < img.height(); ++y) {
            for (unsigned int x = 0; x < img.width(); ++x) {
                img(x, y) = 255;
            }
        }
    }

    for (const auto& ls : lines) {
        const auto dx = ls.p1.x - ls.p0.x;
        const auto dy = ls.p1.y - ls.p0.y;
        const auto len = std::hypot(dx, dy);
        const auto n = 2 * static_cast<int>(std::ceil(len));

        for (int i = 0; i <= n; ++i) {
            const auto t = static_cast<double>(i) / static_cast<double>(n);
            const auto x = static_cast<int>(std::round(ls.p0.x + dx * t));
            const auto y = static_cast<int>(std::round(ls.p0.y + dy * t));
            if (x < 0 || x >= static_cast<int>(img.width()) || y < 0 || y >= static_cast<int>(img.height())) {
                continue;
            }
            img(static_cast<unsigned int>(x), static_cast<unsigned int>(y)) = 0;
        }
    }
}