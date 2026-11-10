// C11 port of kinematics.js. See kinematics.h.
#include "kinematics.h"

#include <math.h>
#include <string.h>

#define STOP_SIGMA 0.004
#define WARN_SIGMA 0.04
#define BASE_DAMPING 0.01
#define MAX_DAMPING 0.15
#define SPEED_FRACTION 0.6
#define LIMIT_RATE 8.0
#define ESCAPE_RATE 1.0
#define ESCAPE_FULL_SPEED 0.05

typedef double Mat3[3][3];

typedef struct {
  double p[3];  // pivot
  double z[3];  // world axis before the joint turns
  Mat3 R;       // frame after the joint turns
} Frame;

// Math.hypot as V8 computes it (scaled, Kahan-compensated sum), so lengths
// round the same way as the JS this was ported from.
static double js_hypot(const double *v, int n) {
  double max = 0;
  for (int i = 0; i < n; i += 1) {
    if (isinf(v[i])) return INFINITY;
    if (isnan(v[i])) return NAN;
    if (fabs(v[i]) > max) max = fabs(v[i]);
  }
  if (max == 0) return 0;
  double sum = 0, compensation = 0;
  for (int i = 0; i < n; i += 1) {
    double r = v[i] / max, summand = r * r - compensation, preliminary = sum + summand;
    compensation = (preliminary - sum) - summand;
    sum = preliminary;
  }
  return sqrt(sum) * max;
}

static void mul(Mat3 a, Mat3 b, Mat3 out) {
  Mat3 t;
  for (int r = 0; r < 3; r += 1)
    for (int c = 0; c < 3; c += 1) t[r][c] = a[r][0] * b[0][c] + a[r][1] * b[1][c] + a[r][2] * b[2][c];
  memcpy(out, t, sizeof t);
}

static void apply(Mat3 m, const double v[3], double out[3]) {
  for (int r = 0; r < 3; r += 1) out[r] = m[r][0] * v[0] + m[r][1] * v[1] + m[r][2] * v[2];
}

// Rodrigues rotation about a (normalised) axis.
static void rotation(const double axis[3], double angle, Mat3 out) {
  double n = js_hypot(axis, 3);
  double x = axis[0] / n, y = axis[1] / n, z = axis[2] / n;
  double c = cos(angle), s = sin(angle), t = 1 - c;
  out[0][0] = t * x * x + c;     out[0][1] = t * x * y - s * z; out[0][2] = t * x * z + s * y;
  out[1][0] = t * x * y + s * z; out[1][1] = t * y * y + c;     out[1][2] = t * y * z - s * x;
  out[2][0] = t * x * z - s * y; out[2][1] = t * y * z + s * x; out[2][2] = t * z * z + c;
}

static void to_quaternion(Mat3 m, double out[4]) {
  double a = m[0][0], b = m[0][1], c = m[0][2], d = m[1][0], e = m[1][1], f = m[1][2], g = m[2][0], h = m[2][1], i = m[2][2];
  double trace = a + e + i, s;
  if (trace > 0) {
    s = sqrt(trace + 1) * 2;
    out[0] = (h - f) / s; out[1] = (c - g) / s; out[2] = (d - b) / s; out[3] = 0.25 * s;
  } else if (a > e && a > i) {
    s = sqrt(1 + a - e - i) * 2;
    out[0] = 0.25 * s; out[1] = (b + d) / s; out[2] = (c + g) / s; out[3] = (h - f) / s;
  } else if (e > i) {
    s = sqrt(1 + e - a - i) * 2;
    out[0] = (b + d) / s; out[1] = 0.25 * s; out[2] = (f + h) / s; out[3] = (c - g) / s;
  } else {
    s = sqrt(1 + i - a - e) * 2;
    out[0] = (c + g) / s; out[1] = (f + h) / s; out[2] = 0.25 * s; out[3] = (d - b) / s;
  }
}

static void frames(const ArmKinematics *k, const double *q, Frame *out) {
  Mat3 R = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, turn;
  double p[3] = {0, 0, 0}, moved[3], z[3];
  for (int i = 0; i < k->n; i += 1) {
    const ArmJoint *joint = &k->joints[i];
    apply(R, joint->origin, moved);
    for (int c = 0; c < 3; c += 1) p[c] = moved[c] + p[c];
    apply(R, joint->axis, z);
    double zn = js_hypot(z, 3);
    rotation(joint->axis, q[i], turn);
    mul(R, turn, R);
    for (int c = 0; c < 3; c += 1) { out[i].p[c] = p[c]; out[i].z[c] = z[c] / zn; }
    memcpy(out[i].R, R, sizeof R);
  }
}

void arm_jacobian(const ArmKinematics *k, const double *q, double jac[6][ARM_MAX_JOINTS]) {
  Frame f[ARM_MAX_JOINTS];
  frames(k, q, f);
  const double *tcp = f[k->n - 1].p;
  memset(jac, 0, sizeof(double) * 6 * ARM_MAX_JOINTS);
  for (int i = 0; i < k->n; i += 1) {
    const double *p = f[i].p, *z = f[i].z;
    double d[3] = {tcp[0] - p[0], tcp[1] - p[1], tcp[2] - p[2]};
    double linear[3] = {z[1] * d[2] - z[2] * d[1], z[2] * d[0] - z[0] * d[2], z[0] * d[1] - z[1] * d[0]};
    for (int r = 0; r < 3; r += 1) { jac[r][i] = linear[r]; jac[r + 3][i] = z[r]; }
  }
}

static double smoothstep(double sigma) {
  if (!isfinite(sigma)) return 0;
  double x = fmin(1, fmax(0, (sigma - STOP_SIGMA) / (WARN_SIGMA - STOP_SIGMA)));
  return x * x * (3 - 2 * x);
}

// Cyclic Jacobi eigendecomposition of a symmetric n x n matrix (row stride
// ARM_MAX_JOINTS); vecs[k][i] is component k of eigenvector i.
static bool eigen(int n, double input[][ARM_MAX_JOINTS], double vecs[][ARM_MAX_JOINTS], double *vals) {
  double a[ARM_MAX_JOINTS][ARM_MAX_JOINTS];
  for (int i = 0; i < n; i += 1)
    for (int j = 0; j < n; j += 1) { a[i][j] = input[i][j]; vecs[i][j] = i == j ? 1 : 0; }
  for (int sweep = 0; sweep < 100; sweep += 1) {
    double off = 0;
    for (int p = 0; p < n; p += 1)
      for (int q = p + 1; q < n; q += 1) off += a[p][q] * a[p][q];
    if (off <= 1e-30) {
      for (int i = 0; i < n; i += 1) vals[i] = a[i][i];
      return true;
    }
    for (int p = 0; p < n; p += 1)
      for (int q = p + 1; q < n; q += 1) {
        double apq = a[p][q];
        if (fabs(apq) <= 1e-300) continue;
        double theta = (a[q][q] - a[p][p]) / (2 * apq);
        double t = (theta >= 0 ? 1 : -1) / (fabs(theta) + sqrt(theta * theta + 1));
        double c = 1 / sqrt(t * t + 1), s = t * c;
        for (int k = 0; k < n; k += 1) { double kp = a[k][p], kq = a[k][q]; a[k][p] = c * kp - s * kq; a[k][q] = s * kp + c * kq; }
        for (int k = 0; k < n; k += 1) { double pk = a[p][k], qk = a[q][k]; a[p][k] = c * pk - s * qk; a[q][k] = s * pk + c * qk; }
        for (int k = 0; k < n; k += 1) { double vp = vecs[k][p], vq = vecs[k][q]; vecs[k][p] = c * vp - s * vq; vecs[k][q] = s * vp + c * vq; }
      }
  }
  return false;
}

static bool locked_axis(int n, const bool *locked, double vecs[][ARM_MAX_JOINTS], int i) {
  for (int k = 0; k < n; k += 1)
    if (locked[k] && fabs(vecs[k][i]) > 0.5) return true;
  return false;
}

// Damped least squares with locked joints held at `fixed` rates. sigma is the
// smallest singular value among the free task directions.
static bool solve(int n, double jac[6][ARM_MAX_JOINTS], const bool *locked, const double *fixed,
                  const double twist[6], bool position_only, double *dq, double *sigma) {
  int rows = position_only ? 3 : 6;
  double j[6][ARM_MAX_JOINTS], residual[6], S[ARM_MAX_JOINTS][ARM_MAX_JOINTS];
  double vecs[ARM_MAX_JOINTS][ARM_MAX_JOINTS], vals[ARM_MAX_JOINTS], ranked[ARM_MAX_JOINTS];
  for (int r = 0; r < 6; r += 1)
    for (int k = 0; k < n; k += 1) j[r][k] = locked[k] || r >= rows ? 0 : jac[r][k];
  memcpy(residual, twist, sizeof residual);
  for (int r = 0; r < rows; r += 1)
    for (int k = 0; k < n; k += 1)
      if (locked[k]) residual[r] -= jac[r][k] * fixed[k];
  for (int a = 0; a < n; a += 1)
    for (int b = 0; b < n; b += 1) {
      double sum = 0;
      for (int r = 0; r < 6; r += 1) sum = sum + j[r][a] * j[r][b];
      S[a][b] = sum;
    }
  if (!eigen(n, S, vecs, vals)) return false;
  int count = 0;
  for (int i = 0; i < n; i += 1)
    if (!locked_axis(n, locked, vecs, i)) ranked[count++] = sqrt(fmax(vals[i], 0));
  int rank = count < rows ? count : rows;
  memcpy(dq, fixed, sizeof(double) * n);
  if (rank == 0) { *sigma = 0; return true; }
  for (int i = 1; i < count; i += 1)  // descending insertion sort
    for (int m = i; m > 0 && ranked[m] > ranked[m - 1]; m -= 1) { double t = ranked[m]; ranked[m] = ranked[m - 1]; ranked[m - 1] = t; }
  *sigma = ranked[rank - 1];
  double untrusted = 1 - smoothstep(*sigma);
  double lambda = BASE_DAMPING + (MAX_DAMPING - BASE_DAMPING) * untrusted * untrusted;
  for (int i = 0; i < n; i += 1) {
    if (locked_axis(n, locked, vecs, i)) continue;
    double numerator = 0;
    for (int r = 0; r < 6; r += 1) {
      double projected = 0;
      for (int c = 0; c < n; c += 1) projected += j[r][c] * vecs[c][i];
      numerator += projected * residual[r];
    }
    double coefficient = numerator / (fmax(vals[i], 0) + lambda * lambda);
    for (int r = 0; r < n; r += 1) dq[r] += coefficient * vecs[r][i];
  }
  return true;
}

// A fully stretched arm cannot move along itself at first order, in either
// direction, so damped least squares alone leaves it stuck there. When the
// position Jacobian is singular along the shoulder-to-tool line and the command
// points back toward the shoulder, return a joint rate that nudges the arm
// toward its rest pose; once it bends, the normal solver follows the command.
static bool escape_drive(const ArmKinematics *k, const double *q, double jac[6][ARM_MAX_JOINTS],
                         const double twist[6], double *drive) {
  Frame f[ARM_MAX_JOINTS];
  frames(k, q, f);
  const double *tcp = f[k->n - 1].p, *shoulder = f[1].p;
  double radial[3] = {tcp[0] - shoulder[0], tcp[1] - shoulder[1], tcp[2] - shoulder[2]};
  double length = js_hypot(radial, 3);
  *drive = 0;
  if (length < 1e-9) return true;
  double unit[3] = {radial[0] / length, radial[1] / length, radial[2] / length};
  double gram[ARM_MAX_JOINTS][ARM_MAX_JOINTS], vecs[ARM_MAX_JOINTS][ARM_MAX_JOINTS], vals[ARM_MAX_JOINTS];
  for (int a = 0; a < 3; a += 1)
    for (int b = 0; b < 3; b += 1) {
      double sum = 0;
      for (int c = 0; c < k->n; c += 1) sum = sum + jac[a][c] * jac[b][c];
      gram[a][b] = sum;
    }
  if (!eigen(3, gram, vecs, vals)) return false;
  int weakest = 0;
  for (int i = 1; i < 3; i += 1)
    if (vals[i] < vals[weakest]) weakest = i;
  double sigma = sqrt(fmax(vals[weakest], 0));
  double along = fabs(vecs[0][weakest] * unit[0] + vecs[1][weakest] * unit[1] + vecs[2][weakest] * unit[2]);
  double inward = -(twist[0] * unit[0] + twist[1] * unit[1] + twist[2] * unit[2]);
  // Only a command that is mostly back toward the shoulder asks to leave.
  if (sigma >= WARN_SIGMA || along < 0.7 || inward < 0.5 * js_hypot(twist, 3)) return true;
  *drive = ESCAPE_RATE * (1 - smoothstep(sigma)) * fmin(1, inward / ESCAPE_FULL_SPEED);
  return true;
}

bool arm_init(ArmKinematics *k, const ArmJoint *joints, int n, const double *rest) {
  if (n < 2 || n > ARM_MAX_JOINTS) return false;
  k->n = n;
  memcpy(k->joints, joints, sizeof(ArmJoint) * n);
  for (int i = 0; i < n; i += 1) k->rest[i] = rest ? rest[i] : 0;
  k->position_only = true;
  return true;
}

bool arm_solve(const ArmKinematics *k, const double *q, ArmPose *out) {
  Frame f[ARM_MAX_JOINTS];
  double jac[6][ARM_MAX_JOINTS], dq[ARM_MAX_JOINTS], sigma;
  bool none[ARM_MAX_JOINTS] = {false};
  double zeros[ARM_MAX_JOINTS] = {0}, still[6] = {0};
  frames(k, q, f);
  for (int i = 0; i < k->n; i += 1) {
    memcpy(&out->links[i * 7], f[i].p, sizeof f[i].p);
    to_quaternion(f[i].R, &out->links[i * 7 + 3]);
  }
  memcpy(out->tcp, f[k->n - 1].p, sizeof out->tcp);
  arm_jacobian(k, q, jac);
  if (!solve(k->n, jac, none, zeros, still, k->position_only, dq, &sigma)) return false;
  out->trust = smoothstep(sigma);
  return true;
}

static double clamp(const ArmKinematics *k, double v, int i) {
  return fmin(k->joints[i].upper, fmax(k->joints[i].lower, v));
}

bool arm_jog(const ArmKinematics *k, const double *q, const double twist[6], double dt, ArmJogStep *out) {
  int n = k->n;
  if (!(dt > 0) || !isfinite(dt)) return false;
  for (int i = 0; i < n; i += 1)
    if (!isfinite(q[i])) return false;
  for (int i = 0; i < 6; i += 1)
    if (!isfinite(twist[i])) return false;

  double current[ARM_MAX_JOINTS], away[ARM_MAX_JOINTS], jac[6][ARM_MAX_JOINTS];
  double fixed[ARM_MAX_JOINTS] = {0}, solved[ARM_MAX_JOINTS], dq[ARM_MAX_JOINTS], sigma = 0, drive;
  bool locked[ARM_MAX_JOINTS] = {false}, speed_limited = false;
  for (int i = 0; i < n; i += 1) current[i] = clamp(k, q[i], i);
  arm_jacobian(k, current, jac);
  if (!escape_drive(k, current, jac, twist, &drive)) return false;
  for (int i = 0; i < n; i += 1) away[i] = k->rest[i] - current[i];
  double away_length = js_hypot(away, n);

  for (int pass = 0; pass < 7; pass += 1) {
    if (!solve(n, jac, locked, fixed, twist, k->position_only, solved, &sigma)) return false;
    if (drive > 0 && away_length > 1e-9)
      for (int i = 0; i < n; i += 1)
        if (!locked[i]) solved[i] += (drive * away[i]) / away_length;
    double shrink = 1;
    for (int i = 0; i < n; i += 1) {
      double cap = k->joints[i].velocity * SPEED_FRACTION;
      if (fabs(solved[i]) > cap) shrink = fmin(shrink, cap / fabs(solved[i]));
    }
    speed_limited = shrink < 1;
    for (int i = 0; i < n; i += 1) dq[i] = speed_limited ? solved[i] * shrink : solved[i];
    bool newly_locked = false;
    for (int i = 0; i < n; i += 1) {
      if (locked[i] || dq[i] == 0) continue;
      bool outward = dq[i] > 0;
      double distance = outward ? k->joints[i].upper - current[i] : current[i] - k->joints[i].lower;
      double cap = fmax(0, fmin(LIMIT_RATE * distance, distance / dt));
      if (fabs(dq[i]) > cap) { locked[i] = true; fixed[i] = outward ? cap : -cap; newly_locked = true; }
    }
    if (!newly_locked) break;
  }

  out->limited = 0;
  for (int i = 0; i < n; i += 1) {
    if (locked[i]) out->limited |= 1 << i;
    out->q[i] = clamp(k, current[i] + dq[i] * dt, i);
  }
  out->flags = (sigma < WARN_SIGMA ? ARM_JOG_NEAR_SINGULARITY : 0) | (out->limited ? ARM_JOG_JOINT_LIMIT : 0)
             | (speed_limited ? ARM_JOG_SPEED_LIMITED : 0) | (drive > 0 ? ARM_JOG_ESCAPING : 0);
  return true;
}
