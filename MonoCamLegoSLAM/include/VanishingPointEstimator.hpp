#pragma once

#include <LineSegmentDetector.hpp>
#include <Camera.hpp>

/**
 * Returns an angle in radians from the forward direction to the right that points to the closest
 * Manhattan grid direction in that quadrant, if one is detected.
 * Line segments that are close to vertical are not considered.
 * If no reliable VPs are found, returns NaN
 */
double estimateVanishingPoint(const std::vector<LineSegment>& lines, const CameraParameters& camParams, unsigned int randomSeed);

// void runTests();
