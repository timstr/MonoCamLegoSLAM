#include <VanishingPointEstimator.hpp>

#include <Assert.hpp>

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <iterator>
#include <random>

// TODO: remove after testing
#include <iostream>

namespace {
    enum class SegmentLabel : std::uint8_t {
        VanishingPoint1,
        VanishingPoint2,
        Outlier
    };

    // std::ostream& operator<<(std::ostream& o, SegmentLabel sl) noexcept {
    //     switch (sl) {
    //     case SegmentLabel::VanishingPoint1:
    //         o << "VP1";
    //         break;
    //     case SegmentLabel::VanishingPoint2:
    //         o << "VP2";
    //         break;
    //     case SegmentLabel::Outlier:
    //         o << "Out";
    //         break;
    //     }
    //     return o;
    // }

    constexpr auto pi = 3.141592653589793238463;

    /**
     * Given a line segment in an image, returns the angle (in radians) to the point on the horizon
     * where that segment's projection intersects the horizon. The angle returned is computed from
     * the point of intersection to a point that is 1 unit away. This acts like a Gnomonic projection
     * in 1D, and extends the real number line to include +/- infinity which are used when line
     * segments are parallel to the horizon.
     * The returned angle is 0 at the center, positive to the right, and negative to the left,
     * in the range [-pi/2, pi/2]
     */
    double viewAngleToHorizonIntersection(const LineSegment& seg, const CameraParameters& camParams) noexcept {
        const auto cw = static_cast<double>(camParams.width);
        const auto ch = static_cast<double>(camParams.height);

        // Maximum horizontal extend is given by tangent of FOV
        const auto xExtent = std::tan(camParams.horizontalFieldOfView / 2.0);
        const auto yExtent = std::tan(camParams.verticalFieldOfView / 2.0);

        // Map segment endpoints from image space to visible part of projective plane
        // Y is flipped
        const auto x0 = xExtent * (2.0 * (seg.p0.x / cw) - 1.0);
        const auto x1 = xExtent * (2.0 * (seg.p1.x / cw) - 1.0);
        const auto y0 = yExtent * (1.0 - 2.0 * (seg.p0.y / ch));
        const auto y1 = yExtent * (1.0 - 2.0 * (seg.p1.y / ch));

        // intersection with x-axis of projective plane is given by alpha/beta
        const auto alpha = x0 * y1 - x1 * y0;
        const auto beta  = y1 - y0;

        // Use atan2(alpha, beta) instead of atan(alpha/beta) to enable parallel lines
        const auto theta = std::atan2(alpha, beta);

        // Wrap around to [-pi/2, pi/2]
        return theta - pi * std::floor(theta / pi + 0.5);
    }

    constexpr auto angularThreshold = pi * 0.2;

    constexpr auto outlierCost = angularThreshold;

    constexpr auto minSegments = std::size_t{3};

    /**
     * Returns a number in [0, 1]
     * 0 -> the angles are perfectly parallel
     * 1 -> the angles are perfectly orthogonal
     * The cost is piecewise linear in both angles
     * NOTE: the orthogonal angle cost is given by 1.0 - parallelAngleCost
     */
    double parallelAngleCost(double theta1, double theta2) noexcept {
        auto d = (theta1 - theta2) / pi;
        return 2.0 * std::abs(d - std::floor(d + 0.5));
    }

} // anonymous namespace

double estimateVanishingPoint(const std::vector<LineSegment>& tentativeLines, const CameraParameters& camParams, unsigned int randomSeed) {

    auto horizonAngles = std::vector<double>{};
    auto labels = std::vector<SegmentLabel>{};
    auto lines = std::vector<LineSegment>{};

    horizonAngles.reserve(tentativeLines.size());
    labels.reserve(tentativeLines.size());
    lines.reserve(tentativeLines.size());

    // TODO: remove after debugging
    auto vertical = std::vector<LineSegment>{};

    for (const auto& tls : tentativeLines) {
        const auto dx = tls.p1.x - tls.p0.x;
        const auto dy = tls.p1.y - tls.p0.y;

        // Discard near-vertical segments
        auto segmentAngle = std::atan2(dy, dx);
        segmentAngle /= pi;
        segmentAngle = std::abs(segmentAngle + 0.5 - std::floor(segmentAngle + 1.0));
        segmentAngle *= pi;
        if (segmentAngle < angularThreshold) {
            vertical.push_back(tls);
            continue;
        }

        // Discard short segments
        if (std::hypot(dx, dy) < 15.0) {
            continue;
        }

        horizonAngles.push_back(viewAngleToHorizonIntersection(tls, camParams));
        labels.push_back(SegmentLabel::VanishingPoint1);
        lines.push_back(tls);
    }

    const auto N = lines.size();
    assert(horizonAngles.size() == N);
    assert(labels.size() == N);

    if (N < minSegments) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    auto prng = std::minstd_rand{};
    prng.seed(randomSeed);

    auto randIndex = std::uniform_int_distribution<std::size_t>{0, lines.size() - 1};
    auto randLabel = std::uniform_int_distribution<std::uint8_t>{1, 2};
    auto randProb  = std::uniform_real_distribution<double>{0.0, 1.0};


    /**
     * The total cost is the sum of:
     *  - the pairwise alignment cost of all segments assigned to VP1
     *  - the pairwise alignment cost of all segments assigned to VP2
     *  - the pairwise orthogonality cost of all VP1->VP2 segment pairs
     *  - the outlier cost times the number of outliers
     * The computational cost of this is quadratic, but the cost of computing the
     * change in cost due to a single label change is linear.
     * The change in cost suffices for optimization using Simulated Annealing, and
     * so the full cost is never computed.
     */

    const auto changeInCost = [&](std::size_t segmentIdx, SegmentLabel newLabel) noexcept -> double {
        const auto currLabel = labels[segmentIdx];
        const auto segAngle = horizonAngles[segmentIdx];
        if (currLabel == newLabel) {
            return 0.0;
        }
        // currLabel != newLabel

        auto delta = 0.0;

        if (newLabel == SegmentLabel::Outlier) {
            delta += outlierCost;
        } else if (currLabel == SegmentLabel::Outlier) {
            delta -= outlierCost;
        }

        for (std::size_t i = 0; i < lines.size(); ++i) {
            if (i == segmentIdx) {
                continue;
            }
            const auto ithLabel = labels[i];
            const auto ithAngle = horizonAngles[i];

            if (ithLabel == SegmentLabel::Outlier) {
                continue;
            }
            assert(ithLabel == SegmentLabel::VanishingPoint1 || ithLabel == SegmentLabel::VanishingPoint2);

            const auto para = parallelAngleCost(ithAngle, segAngle);
            const auto ortho = 1.0 - para;

            if (newLabel == ithLabel) {
                // If ithLabel belongs to a VP and segment joined that VP, add parallel cost
                delta += para;
            } else if (currLabel == ithLabel) {
                // If ithLabel belongs to a VP and segment left that VP, subtract parallel cost
                delta -= para;
            }

            const auto otherLabel = (ithLabel == SegmentLabel::VanishingPoint1) ?
                SegmentLabel::VanishingPoint2 :
                SegmentLabel::VanishingPoint1;
            
            if (newLabel == otherLabel) {
                // If segment joined the other VP, add orthogonal cost
                delta += ortho;
            } else if (currLabel == otherLabel) {
                // If segment left the other VP, subtract orthogonal cost
                delta -= ortho;
            }
        }
    
        return delta;
    };

    auto netCostChange = 0.0;

    // Simulated Annealing
    const auto kMax = std::size_t{100 * lines.size()};
    for (std::size_t k = 0; k < kMax; ++k) {
        const auto T = 1.0 - (static_cast<double>(k) / static_cast<double>(kMax - 1));
        const auto i = randIndex(prng);
        const auto b = randLabel(prng);
        assert(i < labels.size());
        const auto currLabel = labels[i];
        const auto newLabelIdx = (static_cast<std::uint8_t>(currLabel) + b) % 3;
        const auto newLabel = static_cast<SegmentLabel>(newLabelIdx);
        auto deltaCost = changeInCost(i, newLabel);
        // as T tends to zero, P must tend to zero if e_new > e and to a positive value otherwise

        const auto d = 1.0 * T * T * T;
        const auto s = 7.5;

        const auto x = std::exp(-s * deltaCost);
        const auto P = d + ((1.0 - d) * x / (1.0 + x));
        assertWithContext(P >= -1e6 && P <= (1.0 + 1e-6), P, T, deltaCost, x);
        if (P > randProb(prng)) {
            labels[i] = newLabel;
            netCostChange += deltaCost;
        }

        // std::cout << netCostChange << '\n';
    }
    
    // Visualize line segments
    // TODO: remove after testing
    // {
    //     auto vp1 = std::vector<LineSegment>{};
    //     auto vp2 = std::vector<LineSegment>{};
    //     auto outliers = std::vector<LineSegment>{};
    //     for (std::size_t i = 0; i < lines.size(); ++i) {
    //         const auto lbl = labels[i];
    //         const auto& seg = lines[i];
    //         if (lbl == SegmentLabel::VanishingPoint1) {
    //             vp1.push_back(seg);
    //         } else if (lbl == SegmentLabel::VanishingPoint2) {
    //             vp2.push_back(seg);
    //         } else if (lbl == SegmentLabel::Outlier) {
    //             outliers.push_back(seg);
    //         }
    //     }
        
    //     auto img = Image{176, 144};

    //     renderLines(tentativeLines, img);
    //     save_ppm(img, "../temp_all.ppm");

    //     renderLines(vertical, img);
    //     save_ppm(img, "../temp_vertical.ppm");
        
    //     renderLines(vp1, img);
    //     save_ppm(img, "../temp_vp1.ppm");

    //     renderLines(vp2, img);
    //     save_ppm(img, "../temp_vp2.ppm");

    //     renderLines(outliers, img);
    //     save_ppm(img, "../temp_outliers.ppm");
    // }

    // Find the VP with the most assigned line segments, and find its average
    auto nVP1 = std::size_t{0};
    auto nVP2 = std::size_t{0};
    auto sumVP1 = Point2D{};
    auto sumVP2 = Point2D{};
    for (std::size_t i = 0; i < lines.size(); ++i) {
        const auto lbl = labels[i];
        const auto theta = horizonAngles[i] * 2.0;
        if (lbl == SegmentLabel::VanishingPoint1) {
            ++nVP1;
            sumVP1.x += std::cos(theta);
            sumVP1.y += std::sin(theta);
        } else if (lbl == SegmentLabel::VanishingPoint2) {
            ++nVP2;
            sumVP2.x += std::cos(theta);
            sumVP2.y += std::sin(theta);
        }
    }

    const auto moveToRightQuadrant = [](double theta) noexcept {
        const auto t = theta / (0.5 * pi);
        return 0.5 * pi * (t - std::floor(t));
    };

    if (nVP1 >= nVP2 && nVP1 >= minSegments) {
        const auto n = static_cast<double>(nVP1);
        const auto theta = 0.5 * std::atan2(
            sumVP1.y / n,
            sumVP1.x / n
        );
        return moveToRightQuadrant(theta);
    } else if (nVP2 > nVP1 && nVP2 >= minSegments) {
        const auto n = static_cast<double>(nVP2);
        const auto theta = 0.5 * std::atan2(
            sumVP2.y / n,
            sumVP2.x / n
        );
        return moveToRightQuadrant(theta);
    }
    return std::numeric_limits<double>::quiet_NaN();
}

// void runTests() {
//     // 100x100 pixel camera with 90 degree FOV in x and y directions
//     const auto camParams = CameraParameters{100, 100, 0.5 * pi, 0.5 * pi};

//     const auto close = [](double v1, double v2) noexcept {
//         return std::abs(v1 - v2) < 1e-6;
//     };

//     const auto closeMod = [](double v1, double v2, double mod) noexcept {
//         auto d = (v1 - v2) / mod;
//         d = mod * std::abs(d - std::floor(d + 0.5));
//         return std::abs(d) < 1e-6;
//     };

//     {
//         const auto ls = LineSegment{
//             Point2D{ 0.0,  0.0},
//             Point2D{25.0, 25.0},
//             0.0
//         };
//         const auto theta = viewAngleToHorizonIntersection(ls, camParams);
//         assertWithContext(closeMod(theta, 0.0, 0.5 * pi), theta);
//         std::cout << "-";
//     }

//     {
//         const auto ls = LineSegment{
//             Point2D{ 0.0,  0.0},
//             Point2D{50.0, 50.0},
//             0.0
//         };
//         const auto theta = viewAngleToHorizonIntersection(ls, camParams);
//         assertWithContext(closeMod(theta, 0.0, 0.5 * pi), theta);
//         std::cout << "-";
//     }

//     {
//         const auto ls = LineSegment{
//             Point2D{ 0.0,  0.0},
//             Point2D{75.0, 75.0},
//             0.0
//         };
//         const auto theta = viewAngleToHorizonIntersection(ls, camParams);
//         assertWithContext(closeMod(theta, 0.0, 0.5 * pi), theta);
//         std::cout << "-";
//     }

//     {
//         const auto ls = LineSegment{
//             Point2D{60.0, 30.0},
//             Point2D{70.0, 30.0},
//             0.0
//         };
//         const auto theta = viewAngleToHorizonIntersection(ls, camParams);
//         assertWithContext(closeMod(theta, 0.0, 0.5 * pi), theta);
//         std::cout << "-";
//     }

//     {
//         const auto ls = LineSegment{
//             Point2D{ 0.0,  0.0},
//             Point2D{100.0, 0.0},
//             0.0
//         };
//         const auto theta = viewAngleToHorizonIntersection(ls, camParams);
//         assertWithContext(closeMod(theta, 0.0, 0.5 * pi), theta);
//         std::cout << "-";
//     }

//     {
//         const auto ls = LineSegment{
//             Point2D{ 0.0,  0.0},
//             Point2D{100.0, 100.0},
//             0.0
//         };
//         const auto theta = viewAngleToHorizonIntersection(ls, camParams);
//         assertWithContext(closeMod(theta, 0.0, 0.5 * pi), theta);
//         std::cout << "-";
//     }

//     {
//         const auto ls = LineSegment{
//             Point2D{50.0,  0.0},
//             Point2D{62.5, 50.0},
//             0.0
//         };
//         const auto theta = viewAngleToHorizonIntersection(ls, camParams);
//         assertWithContext(closeMod(theta, std::atan2(1.0, 4.0), 0.5 * pi), theta);
//         std::cout << "-";
//     }

//     {
//         const auto ls = LineSegment{
//             Point2D{50.0,   0.0},
//             Point2D{100.0, 100.0},
//             0.0
//         };
//         const auto theta = viewAngleToHorizonIntersection(ls, camParams);
//         assertWithContext(closeMod(theta, std::atan2(1.0, 2.0), 0.5 * pi), theta);
//         std::cout << "-";
//     }

//     {
//         const auto ls = LineSegment{
//             Point2D{50.0,   0.0},
//             Point2D{150.0, 100.0},
//             0.0
//         };
//         const auto theta = viewAngleToHorizonIntersection(ls, camParams);
//         assertWithContext(closeMod(theta, std::atan2(1.0, 1.0), 0.5 * pi), theta);
//         std::cout << "-";
//     }

//     {
//         const auto segs = std::vector<LineSegment>{
//             LineSegment{
//                 Point2D{ 0.0, 0.0},
//                 Point2D{25.0, 25.0},
//                 0.0
//             },
//             LineSegment{
//                 Point2D{0.0, 100.0},
//                 Point2D{25.0, 75.0},
//                 0.0
//             },
//             LineSegment{
//                 Point2D{100.0, 0.0},
//                 Point2D{75.0, 25.0},
//                 0.0
//             },
//             LineSegment{
//                 Point2D{100.0, 100.0},
//                 Point2D{75.0,   75.0},
//                 0.0
//             }
//         };

//         const auto theta = estimateVanishingPoint(segs, camParams, 0);

//         assert(!std::isnan(theta));

//         assert(closeMod(theta, 0.0, 0.5 * pi), theta);
//     }
// }
