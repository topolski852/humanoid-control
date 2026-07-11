#pragma once

// External serial/USB IMU reader (WitMotion protocol) → base orientation for the
// policy. See docs/DAEMON_SPEC.md §9 / HANDOFF §7. The IMU is NOT on the CAN bus;
// a dedicated background thread parses the serial stream and maintains the latest
// orientation. Robot::build_telemetry_json() reads snapshot() and emits the `base`
// block. Nothing here touches CAN or the control loop.
//
// Protocol: WitMotion 11-byte frames  [0x55, type, d0..d7 (4× int16 LE), checksum].
//   type 0x52 = angular velocity (deg/s, ±2000 full scale)
//   type 0x59 = quaternion       (q0..q3, raw/32768; order w,x,y,z)
//   checksum  = sum(bytes[0..9]) & 0xFF
// We only need 0x52 and 0x59; other frame types are validated and skipped.

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

struct ImuConfig {
    bool        enabled       = false;
    std::string device;                       // e.g. /dev/humanoid_imu
    int         baud          = 921600;
    int         staleness_ms  = 100;          // sample older than this → not valid
    // Mounting rotation: quaternion [w,x,y,z] rotating a vector from the IMU frame
    // into the robot base frame. Identity = IMU axes already aligned to base axes.
    std::array<double, 4> mounting_quat = {1.0, 0.0, 0.0, 0.0};
};

// Immutable snapshot of the latest orientation, all in the robot base frame.
struct ImuSample {
    bool                  valid = false;      // fresh (within staleness) AND a quaternion seen
    std::array<double, 4> quaternion = {1, 0, 0, 0};        // w,x,y,z (raw IMU frame, diagnostic)
    std::array<double, 3> angular_velocity = {0, 0, 0};     // rad/s, base frame
    std::array<double, 3> projected_gravity = {0, 0, -1};   // unit gravity, base frame
    uint64_t              last_update_us = 0;
};

class WitMotionReader {
public:
    explicit WitMotionReader(ImuConfig cfg);
    ~WitMotionReader();

    // Open the serial device and start the parse thread. Returns false if the
    // device can't be opened (non-fatal to the daemon — base stays invalid).
    bool start();
    void stop();

    // Thread-safe copy of the latest sample. `valid` reflects staleness at call time.
    ImuSample snapshot() const;

private:
    void run();                               // parse-thread body
    bool open_serial();
    void ingest(uint8_t byte);                // byte-wise resync + frame dispatch
    void handle_frame(const uint8_t* f);      // f = 11 valid, checksummed bytes

    ImuConfig cfg_;
    int       fd_ = -1;

    std::thread        thread_;
    std::atomic<bool>  running_{false};

    mutable std::mutex mtx_;
    // Latest decoded values (base frame), guarded by mtx_.
    std::array<double, 4> quat_{1, 0, 0, 0};
    std::array<double, 3> ang_vel_{0, 0, 0};
    std::array<double, 3> proj_grav_{0, 0, -1};
    uint64_t last_update_us_ = 0;
    bool     have_quat_ = false;

    // Frame-assembly scratch (parse thread only).
    std::array<uint8_t, 11> buf_{};
    size_t                  buf_len_ = 0;
};
