#include <math.h>

struct joysticks {
  float stick_gx;
  float stick_gy;
  float stick_dx;
  float stick_dy;
};

struct CommandJoystick
{
    float roll;
    float pitch;
    float yaw;
    float throttle;
};



CommandJoystick joystick2angle (
    float stick_gx, float stick_gy,
    float stick_dx, float stick_dy,
    float angle_max = 45.0
){
    CommandJoystick cmd;

    cmd.roll = stick_gx * angle_max * M_PI / 180.0f;
    cmd.pitch = -stick_gy * angle_max * M_PI / 180.0f;
    cmd.throttle = (stick_dy * stick_dy * stick_dy) / 100.0f;
    cmd.yaw = 0.0f;

    return cmd;
}