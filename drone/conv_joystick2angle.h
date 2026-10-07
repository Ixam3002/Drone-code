#include <math.h>

struct joysticks {
  float stick_gx;
  float stick_gy;
  float stick_dx;
  float stick_dy;
};

struct command
{
    float roll;
    float pitch;
    float yaw;
    float throttle;
};



CommandJoystick joystick2angle (
    float stick_gx, float stick_gy,
    float stick_dx, float stick_dy,
    float angle_max = 45.0,
){
    command cmd;

    cmd.roll = stick_gx * angle_max * M_PI / 180;
    cmd.pitch = -stick_gy * angle_max * M_PI / 180;
    cmd.throttle = stick_dy**3 * 100;
    cmd.yaw = 0.0;

    return cmd;
};