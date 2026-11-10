// C11 port of src/arm/kinematics.js: forward kinematics and bounded Cartesian
// jogging for a serial arm of revolute joints. Loop order follows the JS (and
// the Rust original before it) so results match kinematics.fixture.json to
// round-off.
#ifndef ARM_KINEMATICS_H
#define ARM_KINEMATICS_H

#include <stdbool.h>

#define ARM_MAX_JOINTS 8

enum {
  ARM_JOG_NEAR_SINGULARITY = 1,
  ARM_JOG_JOINT_LIMIT = 2,
  ARM_JOG_SPEED_LIMITED = 4,
  ARM_JOG_ESCAPING = 8
};

// One revolute joint, with origin rpy zero.
typedef struct {
  double origin[3];
  double axis[3];
  double lower, upper, velocity;
} ArmJoint;

typedef struct {
  int n;
  ArmJoint joints[ARM_MAX_JOINTS];
  double rest[ARM_MAX_JOINTS];
  // Free orientation (the default) constrains only the tool position, which
  // stalls far less often than also holding the wrist orientation.
  bool position_only;
} ArmKinematics;

// Link frames as [px, py, pz, qx, qy, qz, qw] per joint, the tool point, and
// how far the current mode is from a singularity (1 clear, 0 at one).
typedef struct {
  double links[ARM_MAX_JOINTS * 7];
  double tcp[3];
  double trust;
} ArmPose;

typedef struct {
  double q[ARM_MAX_JOINTS];
  int flags;
  int limited;  // bit i set when joint i was held at its limit
} ArmJogStep;

// rest may be NULL for an all-zero rest pose. Returns false if n is out of range.
bool arm_init(ArmKinematics *k, const ArmJoint *joints, int n, const double *rest);

// 6 x n geometric Jacobian of the tool point (the last joint's origin).
void arm_jacobian(const ArmKinematics *k, const double *q, double jac[6][ARM_MAX_JOINTS]);

// Returns false only if the eigensolver fails to converge.
bool arm_solve(const ArmKinematics *k, const double *q, ArmPose *out);

// One bounded jog step toward twist ([vx vy vz wx wy wz]) over dt. It keeps
// every joint inside its limits, damps motion near a singularity and caps
// joint speed. Returns false on non-finite input or dt <= 0.
bool arm_jog(const ArmKinematics *k, const double *q, const double twist[6], double dt, ArmJogStep *out);

#endif
