#include "mbot_odometry.h"
#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

void mbot_calculate_odometry(float vx, float vy, float wz, float dt, float* x, float* y, float* theta) {
    // Update pose
    *x += vx * dt * cos(*theta) - vy * dt * sin(*theta);
    *y += vx * dt * sin(*theta) + vy * dt * cos(*theta);
    *theta += wz * dt;

    // Normalize theta to [-pi, pi]
    while (*theta > PI) *theta -= 2.0 * PI;
    while (*theta <= -PI) *theta += 2.0 * PI;
}
void mbot_update_odometry(float vx, float vy, float wz, float dt, float* x, float* y, float* theta, float gyro_wz_diff){

    *x += vx * dt * cos(*theta) - vy * dt * sin(*theta);
    *y += vx * dt * sin(*theta) + vy * dt * cos(*theta);

    //printf(fabs(gyro_wz_diff));

    if (fabs(wz * dt - gyro_wz_diff) > (0.011)){
        // use gyro angular velocity
        *theta += gyro_wz_diff ;
        //*theta += wz * dt;
    }
    else {
        //use odometry angular velocity
        *theta += wz * dt;
    }

    // Normalize theta to [-pi, pi]
    while (*theta > PI) *theta -= 2.0 * PI;
    while (*theta <= -PI) *theta += 2.0 * PI;
}
