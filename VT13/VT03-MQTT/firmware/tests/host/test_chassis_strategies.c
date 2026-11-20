/* Synthetic geometry tests; none of these dimensions are real vehicle defaults. */
#include <assert.h>
#include <math.h>
#include <string.h>
#include "chassis_strategy.h"

static OmniChassisConfig fixture(void)
{
    const float c = 0.70710678118655f;
    OmniChassisConfig g = {
        .configured = 1, .max_translation_mps = 1, .max_rotation_radps = 1,
        .wheels = {
            {3,  .2f,  .3f, c, -c, .05f, 10},
            {2,  .2f, -.3f, c,  c, .05f, 10},
            {1, -.2f, -.3f, c, -c, .05f, 10},
            {4, -.2f,  .3f, c,  c, .05f, 10},
        }
    };
    return g;
}
static void near(float a, float b) { assert(fabsf(a-b) < 0.002f); }
static void expect_zero(const ChassisKinematicsOutput *o)
{
    assert(o->drive_count == 0 && o->steer_count == 0);
    for (unsigned i=0; i<4; ++i) near(o->drive_speed[i], 0);
}
int main(void)
{
    const ChassisStrategy *mecanum = MecanumChassisStrategy_Get();
    ChassisKinematicsInput in = {.vx=1, .max_drive_speed=100};
    ChassisKinematicsOutput out;
    assert(mecanum->implemented);
    assert(mecanum->compute(&in, &out) == ROBOT_STATUS_OK);
    for (unsigned i=0; i<4; ++i) near(out.drive_speed[i], 50);
    const ChassisStrategy *swerve = SwerveChassisStrategy_Get();
    assert(swerve->execution == CHASSIS_EXECUTION_LEGACY_CONTROLLER);
    assert(swerve->compute(&in, &out) == ROBOT_STATUS_UNSUPPORTED);
    const ChassisStrategy *omni = OmniChassisStrategy_Get();
    assert(omni->implemented && omni->execution == CHASSIS_EXECUTION_KINEMATICS);
    assert(omni->compute(&in, &out) == ROBOT_STATUS_NOT_READY);
    expect_zero(&out);

    OmniChassisConfig g = fixture();
    in.omni = &g; in.max_drive_speed = 10000;
    /* Independent circumference calculation: projection 1/sqrt(2) m/s,
     * wheel circumference pi*0.1 m, ten rotor turns per wheel turn.
     */
    const float n = (1.0f/sqrtf(2.0f)) / (3.14159265358979f*.1f) * 60 * 10;
    assert(omni->compute(&in, &out) == ROBOT_STATUS_OK);
    assert(out.drive_count == 4 && out.steer_count == 0);
    for (unsigned i=0; i<4; ++i) near(out.drive_speed[i], n);
    in.vx=0; in.vy=1;
    assert(omni->compute(&in, &out) == ROBOT_STATUS_OK);
    near(out.drive_speed[0], -n); near(out.drive_speed[1], n);
    near(out.drive_speed[2], -n); near(out.drive_speed[3], n);
    in.vy=0; in.wz=1;
    assert(omni->compute(&in, &out) == ROBOT_STATUS_OK);
    near(out.drive_speed[0], -.5f*n); near(out.drive_speed[1], .5f*n);
    near(out.drive_speed[2], .5f*n); near(out.drive_speed[3], -.5f*n);
    in.vx=.3f; in.vy=-.4f; in.wz=.5f;
    assert(omni->compute(&in, &out) == ROBOT_STATUS_OK);
    float expected[4] = {.45f*n, .15f*n, .95f*n, -.35f*n};
    for (unsigned i=0; i<4; ++i) near(out.drive_speed[i], expected[i]);
    in.max_drive_speed = 200;
    assert(omni->compute(&in, &out) == ROBOT_STATUS_OK);
    for (unsigned i=0; i<4; ++i) near(out.drive_speed[i], expected[i]*200/(.95f*n));

    in.max_drive_speed=10000; in.vx=-.3f; in.vy=.4f; in.wz=-.5f;
    assert(omni->compute(&in, &out) == ROBOT_STATUS_OK);
    for (unsigned i=0; i<4; ++i) near(out.drive_speed[i], -expected[i]);
    in.vx=1; in.vy=1; in.wz=0; /* Diagonal translation has the same 1 m/s cap. */
    assert(omni->compute(&in, &out) == ROBOT_STATUS_OK);
    near(out.drive_speed[0], 0); near(out.drive_speed[1], sqrtf(2)*n);
    in.vx=0; in.vy=0; in.wz=0;
    assert(omni->compute(&in, &out) == ROBOT_STATUS_OK);
    for (unsigned i=0; i<4; ++i) near(out.drive_speed[i], 0);

    in.vx=1; g.wheels[0].radius_m *= 2;
    g.wheels[1].reduction_ratio *= 2;
    assert(omni->compute(&in, &out) == ROBOT_STATUS_OK);
    near(out.drive_speed[0], n/2); near(out.drive_speed[1], 2*n);
    OmniWheelConfig tmp=g.wheels[0]; g.wheels[0]=g.wheels[2]; g.wheels[2]=tmp;
    assert(omni->compute(&in, &out) == ROBOT_STATUS_OK);
    near(out.drive_speed[0], n); near(out.drive_speed[2], n/2);
    in.max_drive_speed=0;
    assert(omni->compute(&in, &out) == ROBOT_STATUS_OK);
    for (unsigned i=0; i<4; ++i) near(out.drive_speed[i], 0);
    in.max_drive_speed=10000;

    /* Every rejection must clear a previously nonzero output. */
    for (unsigned bad=0; bad<11; ++bad) {
        g=fixture(); in.vx=1;
        switch (bad) {
        case 0: g.wheels[0].radius_m=0; break;
        case 1: g.wheels[2].reduction_ratio=-1; break;
        case 2: g.wheels[0].drive_x=0; break;
        case 3: g.wheels[1].motor_id=g.wheels[0].motor_id; break;
        case 4: in.vx=NAN; break;
        case 5: g.wheels[3].y_m=INFINITY; break;
        case 6: g.max_translation_mps=0; break;
        case 7: g.max_rotation_radps=NAN; break;
        case 8:
            for (unsigned i=0;i<4;++i) {g.wheels[i].drive_x=1; g.wheels[i].drive_y=0;}
            break; /* No sideways DOF. */
        case 9:
            for (unsigned i=0;i<4;++i) {g.wheels[i].x_m=0; g.wheels[i].y_m=0;}
            break; /* No rotational DOF. */
        case 10: g.wheels[0].radius_m=1e-38f; break; /* Reject RPM overflow. */
        }
        memset(&out, 0x5a, sizeof(out));
        assert(omni->compute(&in, &out) == ROBOT_STATUS_INVALID_ARGUMENT);
        expect_zero(&out);
    }
    g=fixture(); g.configured=0;
    assert(omni->compute(&in,&out)==ROBOT_STATUS_NOT_READY); expect_zero(&out);
    assert(omni->compute(NULL,&out)==ROBOT_STATUS_INVALID_ARGUMENT); expect_zero(&out);
    assert(omni->compute(&in,NULL)==ROBOT_STATUS_INVALID_ARGUMENT);
    return 0;
}
