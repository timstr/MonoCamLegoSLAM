#pragma once

#include <Image.hpp>

#include <vector>

class Point2D {
public:
    Point2D(double _x = 0.0, double _y = 0.0) noexcept;

    double x;
    double y;
};

class LineSegment {
public:
    LineSegment(Point2D _p0, Point2D _p1, double _width) noexcept;

    Point2D p0;
    Point2D p1;
    double width;
};

std::vector<LineSegment> detectLines(const Image& img, double minLength);

void renderLines(const std::vector<LineSegment>& lines, Image& dst, bool eraseDst = true);
