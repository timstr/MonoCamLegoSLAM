#include <Camera.hpp>

#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <string.h>

#include <linux/videodev2.h>

#include <cassert>
#include <iostream>
#include <utility>

void errno_exit(const char *s){
    std::cerr << s << "error " << errno << strerror(errno) << std::endl;
    std::exit(EXIT_FAILURE);
}

int xioctl(int fh, int request, void *arg){
    while (true) {
        auto r = ioctl(fh, request, arg);
        if (r != -1 || errno != EINTR) {
            return r;
        }
    }
}

void process_image(const void *p, int size, const ImageMemoryAccess& ima, Image& dst){
    assert(p);

    dst.resize(ima.width, ima.height);

    auto pb = reinterpret_cast<const std::uint8_t*>(p);
    for (unsigned int y = 0; y < ima.height; ++y) {
        const auto pRow = pb + (y * ima.stride);
        for (unsigned int x = 0; x < ima.width; ++x) {
            // NOTE: skip-counting to grab only luminance data, assuming YUYV format
            const auto pPix = pRow + (2 * x);
            assert((pPix - pb) < size);
            dst(x, y) = *pPix;
        }
    }
}

bool try_read_frame(int fd, std::vector<DataSpan>& buffers, const ImageMemoryAccess& ima, Image& dst){
    auto buf = v4l2_buffer{};

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
        switch (errno) {
        case EAGAIN:
            return false;

        case EIO:
            /* Could ignore EIO, see spec. */

            /* fall through */

        default:
            errno_exit("VIDIOC_DQBUF");
        }
    }

    assert(buf.index < buffers.size());

    process_image(buffers[buf.index].start, buf.bytesused, ima, dst);

    if (-1 == xioctl(fd, VIDIOC_QBUF, &buf)) {
        errno_exit("VIDIOC_QBUF");
    }

    return true;
}

void read_next_frame(int fd, std::vector<DataSpan>& buffers, const ImageMemoryAccess& ima, Image& dst){
    while (true) {
        fd_set fds;
        int r;

        FD_ZERO(&fds);
        FD_SET(fd, &fds);

        /* Timeout. */
        auto tv = timeval{};
        tv.tv_sec = 10;
        tv.tv_usec = 0;

        r = select(fd + 1, &fds, NULL, NULL, &tv);

        if (-1 == r) {
            if (EINTR == errno) {
                continue;
            }
            errno_exit("select");
        }

        if (0 == r) {
            std::cerr << "select timeout" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        if (try_read_frame(fd, buffers, ima, dst)) {
            break;
        }
        /* EAGAIN - continue select loop. */
    }
}

void stop_capturing(int fd){
    enum v4l2_buf_type type;

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type)) {
        errno_exit("VIDIOC_STREAMOFF");
    }
}

void start_capturing(int fd, std::vector<DataSpan>& buffers){
    for (unsigned int i = 0; i < buffers.size(); ++i) {
        auto buf = v4l2_buffer{};

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf)) {
            errno_exit("VIDIOC_QBUF");
        }
    }
    auto type = v4l2_buf_type{V4L2_BUF_TYPE_VIDEO_CAPTURE};
    if (-1 == xioctl(fd, VIDIOC_STREAMON, &type)) {
        errno_exit("VIDIOC_STREAMON");
    }
}

void uninit_device(std::vector<DataSpan>& buffers){
    for (unsigned int i = 0; i < buffers.size(); ++i) {
        if (-1 == munmap(buffers[i].start, buffers[i].length)) {
            errno_exit("munmap");
        }
    }

    buffers.clear();
}

std::vector<DataSpan> init_mmap(int fd){
    auto req = v4l2_requestbuffers{};

    req.count = 1;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
        if (EINVAL == errno) {
            std::cerr << "Memory mapping is not supported" << std::endl;
            std::exit(EXIT_FAILURE);
        } else {
            errno_exit("VIDIOC_REQBUFS");
        }
    }

    if (req.count < 1) {
        std::cerr << "Insufficient buffer memory on device" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    std::cout << "Requested 1 buffer, got " << req.count << '\n';

    auto buffers = std::vector<DataSpan>{};

    for (unsigned int i = 0; i < req.count; ++i) {
        auto iobuf = v4l2_buffer{};

        iobuf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        iobuf.memory      = V4L2_MEMORY_MMAP;
        iobuf.index       = i;

        if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &iobuf)) {
            errno_exit("VIDIOC_QUERYBUF");
        }

        auto mybuf = DataSpan{
            mmap(
                NULL /* start anywhere */,
                iobuf.length,
                PROT_READ | PROT_WRITE /* required */,
                MAP_SHARED /* recommended */,
                fd,
                iobuf.m.offset
            ),
            iobuf.length
        };

        if (mybuf.start == MAP_FAILED) {
            errno_exit("mmap");
        }

        buffers.push_back(mybuf);
    }

    return buffers;
}

ImageMemoryAccess get_image_format(int fd) {
    auto fmt = v4l2_format{};

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width       = 640;
    fmt.fmt.pix.height      = 480;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;

    if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt)) {
        errno_exit("VIDIOC_S_FMT");
    }

    if (-1 == xioctl(fd, VIDIOC_G_FMT, &fmt)) {
        errno_exit("VIDIOC_G_FMT");
    }

    auto ima = ImageMemoryAccess{
        fmt.fmt.pix.width,
        fmt.fmt.pix.height,
        fmt.fmt.pix.bytesperline
    };

    return ima;
}

std::vector<DataSpan> init_device(int fd){
    auto cap = v4l2_capability{};

    if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
        if (EINVAL == errno) {
            std::cerr << "Device is not a V4L2-compatible device" << std::endl;
            std::exit(EXIT_FAILURE);
        } else {
            errno_exit("VIDIOC_QUERYCAP");
        }
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        std::cerr <<"Device is not a video capture device" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        std::cerr << "Device does not support streaming i/o" << std::endl;
        std::exit(EXIT_FAILURE);
    }


    /* Select video input, video standard and tune here. */


    auto cropcap = v4l2_cropcap{};

    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap)) {
        auto crop = v4l2_crop{};
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default */

        if (-1 == xioctl(fd, VIDIOC_S_CROP, &crop)) {
            switch (errno) {
            case EINVAL:
                /* Cropping not supported. */
                break;
            default:
                /* Errors ignored. */
                break;
            }
        }
    } else {
        /* Errors ignored. */
    }

    
    return init_mmap(fd);
}

void close_device(int fd){
    if (-1 == close(fd)) {
        errno_exit("close");
    }
}

int open_device(const std::string& dev_name){
    struct stat st = {}; // NOTE: 'struct' is needed here to disambiguate with a function called stat. Grumble grumble grumble...

    if (-1 == stat(dev_name.c_str(), &st)) {
        std::cerr << "Cannot identify '" << dev_name << "': " << errno << ", " << strerror(errno) << std::endl;
        std::exit(EXIT_FAILURE);
    }

    if (!S_ISCHR(st.st_mode)) {
        std::cerr << dev_name << " is no device" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    auto fd = open(dev_name.c_str(), O_RDWR /* required */ | O_NONBLOCK, 0);

    if (fd == -1) {
        std::cerr << "Cannot open '" << dev_name << "': " << errno << ", " << strerror(errno) << std::endl;
        std::exit(EXIT_FAILURE);
    }

    return fd;
}

DataSpan::DataSpan(void* data, std::size_t len) noexcept
    : start(data)
    , length(len) {

}

ImageMemoryAccess::ImageMemoryAccess(unsigned int w, unsigned int h, unsigned int s) noexcept
    : width(w)
    , height(h)
    , stride(s) {
    
}

CameraParameters::CameraParameters(unsigned int w, unsigned int h, double fovX, double fovY) noexcept
    : width(w)
    , height(h)
    , horizontalFieldOfView(fovX)
    , verticalFieldOfView(fovY) {
    
}

Camera::Camera(const std::string& file)
    : m_fd(open_device(file))
    , m_ima(get_image_format(m_fd))
    , m_buffers(init_device(m_fd)) {
    start_capturing(m_fd, m_buffers);
}

Camera::Camera(Camera&& c) noexcept
    : m_fd(c.m_fd)
    , m_ima(c.m_ima)
    , m_buffers(std::move(c.m_buffers)) {
    c.m_fd = -1;
}

Camera::~Camera() noexcept {
    if (m_fd == -1) {
        return;
    }
    stop_capturing(m_fd);
    uninit_device(m_buffers);
    close_device(m_fd);
}

bool Camera::capture(Image& dst) {
    if (m_fd == -1) {
        return false;
    }
    read_next_frame(m_fd, m_buffers, m_ima, dst);
    return true;
}

