#pragma once

#include <Image.hpp>

#include <cstdint>
#include <vector>

struct DataSpan {
    DataSpan(void* data, std::size_t len) noexcept;

    void   *start;
    size_t  length;
};

struct ImageMemoryAccess {
    ImageMemoryAccess(unsigned int w, unsigned int h, unsigned int s) noexcept;

    unsigned int width;
    unsigned int height;
    unsigned int stride;
};

struct CameraParameters {
    CameraParameters(unsigned int w, unsigned int h, double fovX, double fovY) noexcept;

    unsigned int width;
    unsigned int height;
    double horizontalFieldOfView;
    double verticalFieldOfView;
};

class Camera {
public:
    Camera(const std::string& file = "/dev/video0");
    Camera(Camera&&) noexcept;
    ~Camera() noexcept;

    Camera(const Camera&) = delete;
    Camera& operator=(const Camera&) = delete;
    Camera& operator=(Camera&&) = delete;

    bool capture(Image& dst);

private:
    // NOTE: the order of the following members must not be changed
    int m_fd;
    ImageMemoryAccess m_ima;
    std::vector<DataSpan> m_buffers;
};
