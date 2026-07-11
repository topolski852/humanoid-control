// Standalone IMU reader smoke test — verifies WitMotionReader against real hardware
// WITHOUT starting the daemon or touching CAN. Prints the `base` values the daemon
// would emit. Build: `make imu-test`  →  run: `./build/imu_test [device] [baud]`.

#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <chrono>

#include "imu/witmotion_reader.hpp"

int main(int argc, char* argv[]) {
    ImuConfig cfg;
    cfg.enabled = true;
    cfg.device  = (argc > 1) ? argv[1] : "/dev/ttyUSB0";
    cfg.baud    = (argc > 2) ? atoi(argv[2]) : 921600;

    WitMotionReader reader(cfg);
    if (!reader.start()) {
        fprintf(stderr, "imu_test: reader failed to start\n");
        return 1;
    }

    for (int i = 0; i < 20; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        ImuSample s = reader.snapshot();
        printf("valid=%d  quat=[% .4f % .4f % .4f % .4f]  ang_vel=[% .3f % .3f % .3f] rad/s  "
               "proj_grav=[% .3f % .3f % .3f]\n",
               s.valid,
               s.quaternion[0], s.quaternion[1], s.quaternion[2], s.quaternion[3],
               s.angular_velocity[0], s.angular_velocity[1], s.angular_velocity[2],
               s.projected_gravity[0], s.projected_gravity[1], s.projected_gravity[2]);
    }

    reader.stop();
    return 0;
}
