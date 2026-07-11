#include "imu/witmotion_reader.hpp"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>

#include <chrono>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace {

constexpr double kPi        = 3.14159265358979323846;
constexpr double kDeg2Rad   = kPi / 180.0;
constexpr double kGyroScale = 2000.0;   // ±2000 deg/s full scale (raw/32768)

uint64_t now_us() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

// Rotate v by the inverse of quaternion q = [w,x,y,z]. Byte-for-byte identical to
// humanoid_control.base_state.quat_rotate_inverse so daemon-computed projected_gravity
// matches what the Python side would compute from the same quaternion.
std::array<double, 3> quat_rotate_inverse(const std::array<double, 4>& q,
                                          const std::array<double, 3>& v) {
    const double w  = q[0];
    const double qx = q[1], qy = q[2], qz = q[3];
    const double two_w2m1 = 2.0 * w * w - 1.0;
    std::array<double, 3> a = {v[0] * two_w2m1, v[1] * two_w2m1, v[2] * two_w2m1};
    // b = cross(q_vec, v) * w * 2
    std::array<double, 3> cross = {
        qy * v[2] - qz * v[1],
        qz * v[0] - qx * v[2],
        qx * v[1] - qy * v[0],
    };
    std::array<double, 3> b = {cross[0] * w * 2.0, cross[1] * w * 2.0, cross[2] * w * 2.0};
    const double dot = qx * v[0] + qy * v[1] + qz * v[2];
    std::array<double, 3> c = {qx * dot * 2.0, qy * dot * 2.0, qz * dot * 2.0};
    return {a[0] - b[0] + c[0], a[1] - b[1] + c[1], a[2] - b[2] + c[2]};
}

// Rotate vector v by quaternion q = [w,x,y,z] (v' = q v q*). Used for mounting.
std::array<double, 3> quat_rotate(const std::array<double, 4>& q,
                                  const std::array<double, 3>& v) {
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    // t = 2 * cross(q_vec, v)
    std::array<double, 3> t = {
        2.0 * (y * v[2] - z * v[1]),
        2.0 * (z * v[0] - x * v[2]),
        2.0 * (x * v[1] - y * v[0]),
    };
    // v' = v + w*t + cross(q_vec, t)
    return {
        v[0] + w * t[0] + (y * t[2] - z * t[1]),
        v[1] + w * t[1] + (z * t[0] - x * t[2]),
        v[2] + w * t[2] + (x * t[1] - y * t[0]),
    };
}

// Map a numeric baud to the termios speed_t constant. Returns B0 if unsupported.
speed_t baud_to_speed(int baud) {
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default:     return B0;
    }
}

}  // namespace

WitMotionReader::WitMotionReader(ImuConfig cfg) : cfg_(std::move(cfg)) {}

WitMotionReader::~WitMotionReader() { stop(); }

bool WitMotionReader::open_serial() {
    speed_t speed = baud_to_speed(cfg_.baud);
    if (speed == B0) {
        fprintf(stderr, "[imu] unsupported baud %d\n", cfg_.baud);
        return false;
    }

    fd_ = ::open(cfg_.device.c_str(), O_RDONLY | O_NOCTTY);
    if (fd_ < 0) {
        fprintf(stderr, "[imu] cannot open %s: %s\n", cfg_.device.c_str(), strerror(errno));
        return false;
    }

    struct termios tio{};
    if (tcgetattr(fd_, &tio) != 0) {
        fprintf(stderr, "[imu] tcgetattr failed: %s\n", strerror(errno));
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CRTSCTS;
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);
    // Blocking read that returns as soon as ≥1 byte is available (VMIN=1, VTIME=0),
    // so the parse thread never busy-spins.
    tio.c_cc[VMIN]  = 1;
    tio.c_cc[VTIME] = 0;
    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
        fprintf(stderr, "[imu] tcsetattr failed: %s\n", strerror(errno));
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    tcflush(fd_, TCIFLUSH);
    return true;
}

bool WitMotionReader::start() {
    if (running_.load()) return true;
    if (!open_serial()) return false;
    running_.store(true);
    thread_ = std::thread(&WitMotionReader::run, this);
    fprintf(stderr, "[imu] reader started on %s @ %d baud\n", cfg_.device.c_str(), cfg_.baud);
    return true;
}

void WitMotionReader::stop() {
    if (!running_.exchange(false)) {
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
        return;
    }
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }   // unblock the read()
    if (thread_.joinable()) thread_.join();
}

void WitMotionReader::run() {
    uint8_t chunk[256];
    while (running_.load()) {
        ssize_t n = ::read(fd_, chunk, sizeof(chunk));
        if (n < 0) {
            if (errno == EINTR) continue;
            if (running_.load())
                fprintf(stderr, "[imu] read error: %s\n", strerror(errno));
            break;
        }
        if (n == 0) continue;
        for (ssize_t i = 0; i < n; ++i) ingest(chunk[i]);
    }
}

// Sliding-window resync: accumulate bytes until a valid 11-byte, checksummed frame
// starting with 0x55 is assembled; on mismatch drop the oldest byte and retry.
void WitMotionReader::ingest(uint8_t byte) {
    if (buf_len_ == 0) {
        if (byte != 0x55) return;          // wait for a header
        buf_[buf_len_++] = byte;
        return;
    }
    buf_[buf_len_++] = byte;
    if (buf_len_ < buf_.size()) return;

    // Have 11 bytes. Validate checksum.
    unsigned sum = 0;
    for (int i = 0; i < 10; ++i) sum += buf_[i];
    if ((sum & 0xFF) == buf_[10]) {
        handle_frame(buf_.data());
        buf_len_ = 0;
    } else {
        // Bad frame — resync by discarding the first byte and re-scanning for 0x55.
        size_t next = 1;
        while (next < buf_.size() && buf_[next] != 0x55) ++next;
        buf_len_ = buf_.size() - next;
        std::memmove(buf_.data(), buf_.data() + next, buf_len_);
    }
}

void WitMotionReader::handle_frame(const uint8_t* f) {
    auto i16 = [](const uint8_t* p) -> int16_t {
        return static_cast<int16_t>(static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8));
    };
    const uint8_t type = f[1];

    if (type == 0x52) {                        // angular velocity (deg/s → rad/s)
        std::array<double, 3> w_imu = {
            i16(f + 2) / 32768.0 * kGyroScale * kDeg2Rad,
            i16(f + 4) / 32768.0 * kGyroScale * kDeg2Rad,
            i16(f + 6) / 32768.0 * kGyroScale * kDeg2Rad,
        };
        std::array<double, 3> w_base = quat_rotate(cfg_.mounting_quat, w_imu);
        std::lock_guard<std::mutex> lk(mtx_);
        ang_vel_ = w_base;
        last_update_us_ = now_us();
    } else if (type == 0x59) {                 // quaternion (raw/32768; w,x,y,z)
        std::array<double, 4> q = {
            i16(f + 2) / 32768.0,
            i16(f + 4) / 32768.0,
            i16(f + 6) / 32768.0,
            i16(f + 8) / 32768.0,
        };
        // gravity in IMU frame, then rotate into base frame via mounting.
        std::array<double, 3> pg_imu  = quat_rotate_inverse(q, {0.0, 0.0, -1.0});
        std::array<double, 3> pg_base = quat_rotate(cfg_.mounting_quat, pg_imu);
        std::lock_guard<std::mutex> lk(mtx_);
        quat_ = q;
        proj_grav_ = pg_base;
        have_quat_ = true;
        last_update_us_ = now_us();
    }
    // Other frame types (time/accel/angle/mag/pressure) are valid but unused here.
}

ImuSample WitMotionReader::snapshot() const {
    ImuSample s;
    std::lock_guard<std::mutex> lk(mtx_);
    s.quaternion        = quat_;
    s.angular_velocity  = ang_vel_;
    s.projected_gravity = proj_grav_;
    s.last_update_us    = last_update_us_;
    const uint64_t age_us = now_us() - last_update_us_;
    s.valid = have_quat_ && last_update_us_ != 0 &&
              age_us <= static_cast<uint64_t>(cfg_.staleness_ms) * 1000ull;
    return s;
}
