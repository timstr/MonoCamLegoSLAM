#include <fstream>
#include <iostream>

#include <LineSegmentDetector.hpp>
#include <VanishingPointEstimator.hpp>
#include <Assert.hpp>

namespace {
    constexpr auto pi = 3.141592653589793238463;
} // anonymous namespace

std::vector<LineSegment> readLines(std::ifstream& inFile) {
    // Assumed format per line (taken from lsd demo program):
    // x1 y1 x2 y2 width p -log10(NFA)

    auto out = std::vector<LineSegment>{};

    auto x1 = double{};
    auto y1 = double{};
    auto x2 = double{};
    auto y2 = double{};
    auto w = double{};
    auto p = double{};
    auto mlog10NFA = double{};

    while (inFile >> x1 >> y1 >> x2 >> y2 >> w >> p >> mlog10NFA) {
        out.push_back(LineSegment{
            Point2D{x1, y1},
            Point2D{x2, y2},
            w
        });
    }

    return out;
}

int main(int argc, char** argv){
    if (argc != 2) {
        std::cout << "Usage: vp_batch path/to/lines.txt\n";
        return 1;
    }

    const auto camParams = CameraParameters{
        176,       // Width,  taken from output files
        144,       // Height, taken from output files
        pi * 0.25, // (45 degrees) horizontal FOV, manually measured
        pi * 0.2   // (36 degrees) vertical FOV, manually measured
    };
    
    auto f = std::ifstream{argv[1]};
    assert(f.is_open());
    auto segs = readLines(f);
    // TODO: try using different random seeds?
    auto gridAngle = estimateVanishingPoint(segs, camParams, 0);
    std::cout << gridAngle << '\n';
}
