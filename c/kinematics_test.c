// The C kinematics against the same checks as ../kinematics.test.js: the
// fixtures recorded from the retired Rust/WASM build, the Python FK oracle,
// and the behavioural tests.
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "kinematics.h"
#include "build/fixture.h"

static int failures = 0, checks = 0;

#define CHECK(cond, ...)                                  \
  do {                                                    \
    checks += 1;                                          \
    if (!(cond)) {                                        \
      if (failures++ < 20) {                              \
        printf("  FAIL  %s:%d  ", __FILE__, __LINE__);    \
        printf(__VA_ARGS__);                              \
        printf("\n");                                     \
      }                                                   \
    }                                                     \
  } while (0)

static void close_to(const double *actual, const double *expected, int n, double tol, const char *label) {
  for (int i = 0; i < n; i += 1)
    CHECK(fabs(actual[i] - expected[i]) <= tol, "%s[%d]: %.17g vs %.17g", label, i, actual[i], expected[i]);
}

static const double HOME[6] = {0, -0.45, 0.95, 0, 0.55, 0};

static ArmKinematics engine(bool position_only) {
  ArmKinematics k;
  arm_init(&k, GLUON, GLUON_JOINTS, HOME);
  k.position_only = position_only;
  return k;
}

static void fk_matches_fixture(void) {
  CHECK(GLUON_JOINTS == 6, "expected 6 revolute joints, got %d", GLUON_JOINTS);
  for (int r = 0; r < FK_ROWS; r += 1)
    for (int mode = 0; mode < 2; mode += 1) {
      ArmKinematics k = engine(mode == 0);
      ArmPose out;
      CHECK(arm_solve(&k, FK[r].q, &out), "solve failed");
      close_to(out.links, FK[r].links, 42, 1e-12, "links");
      close_to(out.tcp, FK[r].tcp, 3, 1e-12, "tcp");
      close_to(&out.trust, &FK[r].trust[mode], 1, 1e-9, "trust");
    }
}

static void fk_matches_python_oracle(void) {
  static const struct { double q[6], position[3], rotation[3][3]; } poses[] = {
    {{0, -0.45, 0.95, 0, 0.55, 0}, {0.075661948, -0.326696982, 0.305343404},
     {{0.852524522060, 0.522687228931, 0}, {-0.088839654932, 0.144901157267, -0.985449729988}, {-0.515081988618, 0.840120060072, 0.169967142900}}},
    {{0, 0, 0, 0, 0, 0}, {0.075685673, -0.000022696, 0.533979000}, {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}},
    {{0.3, -0.5, 0.8, 0.2, -0.4, 0.6}, {-0.023166359, -0.331041311, 0.341080001},
     {{0.932123466540, -0.056435865160, -0.357716139207}, {-0.103442787863, 0.905134673347, -0.412347926809}, {0.347052492808, 0.421362333661, 0.837859386178}}},
    {{-1, 0.7, -1.2, 0.5, 1, -0.8}, {-0.238108309, 0.242822664, 0.122997961},
     {{-0.230202307050, 0.142598520700, -0.962638332772}, {0.789903817462, 0.605145252162, -0.099253125611}, {0.568382667740, -0.783239992390, -0.251944949014}}},
  };
  for (size_t p = 0; p < sizeof poses / sizeof poses[0]; p += 1) {
    ArmKinematics k = engine(true);
    ArmPose out;
    arm_solve(&k, poses[p].q, &out);
    close_to(out.tcp, poses[p].position, 3, 1e-6, "oracle tcp");
    double x = out.links[38], y = out.links[39], z = out.links[40], w = out.links[41];
    double R[3][3] = {
      {1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)},
      {2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)},
      {2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)},
    };
    for (int r = 0; r < 3; r += 1) close_to(R[r], poses[p].rotation[r], 3, 1e-9, "oracle rotation");
  }
}

static void jacobian_matches_central_differences(void) {
  const double q[6] = {0.2, -0.3, 0.6, 0.4, -0.5, 0.1}, eps = 1e-6;
  ArmKinematics k = engine(true);
  double J[6][ARM_MAX_JOINTS];
  arm_jacobian(&k, q, J);
  for (int i = 0; i < 6; i += 1) {
    double plus[6], minus[6];
    ArmPose a, b;
    memcpy(plus, q, sizeof q);
    memcpy(minus, q, sizeof q);
    plus[i] += eps;
    minus[i] -= eps;
    arm_solve(&k, plus, &a);
    arm_solve(&k, minus, &b);
    for (int r = 0; r < 3; r += 1)
      CHECK(fabs(J[r][i] - (a.tcp[r] - b.tcp[r]) / (2 * eps)) <= 1e-5, "jacobian column %d row %d", i, r);
  }
}

// Replays a recorded run and checks each checkpoint against the Rust fixture.
// Comparison stops at the stuck-arm escape, the one intended difference.
// Returns true if the run escaped.
static bool replay(const JogRun *r, int steps, bool varying) {
  static const double dts[4] = {1.0 / 60, 1.0 / 144, 1.0 / 30, 0.05};
  ArmKinematics k = engine(r->position_only);
  double q[6];
  int flags_or = 0, c = 0;
  memcpy(q, r->start, sizeof q);
  for (int n = 1; n <= steps; n += 1) {
    int f = n - 1;
    double twist[6];
    memcpy(twist, r->twist, sizeof twist);
    double dt = JOG_DT;
    if (varying) {
      twist[0] = 0.35 * cos(f / 90.0);
      twist[1] = 0.35 * sin(f / 70.0);
      twist[2] = 0.2 * sin(f / 50.0);
      dt = dts[f % 4];
    }
    ArmJogStep step;
    CHECK(arm_jog(&k, q, twist, dt, &step), "jog rejected finite input");
    if (step.flags & ARM_JOG_ESCAPING) return true;
    memcpy(q, step.q, sizeof q);
    flags_or |= step.flags;
    if (c < r->count && r->checkpoints[c].step == n) {
      const Checkpoint *want = &r->checkpoints[c++];
      close_to(step.q, want->q, 6, 1e-9, "jog q");
      CHECK(step.flags == want->flags && step.limited == want->limited,
            "step %d flags/limited %d/%d vs %d/%d", n, step.flags, step.limited, want->flags, want->limited);
    }
  }
  CHECK(c == r->count, "reached %d of %d checkpoints", c, r->count);
  CHECK(flags_or == r->flags_or, "flagsOr %d vs %d", flags_or, r->flags_or);
  return false;
}

static void jog_matches_fixture(void) {
  int escaped = 0;
  for (int r = 0; r < JOG_ROWS; r += 1) escaped += replay(&JOG[r], 600, false);
  for (int r = 0; r < VARYING_ROWS; r += 1) escaped += replay(&VARYING[r], 2400, true);
  // Four runs start at the straight pose and are commanded back toward the
  // shoulder, and one long run wanders into it. Every other run is pinned in full.
  CHECK(escaped == 5, "escaped %d runs, expected 5", escaped);
}

static void stretched_arm_comes_back(void) {
  for (int mode = 0; mode < 2; mode += 1) {
    ArmKinematics k = engine(mode == 0);
    double q[6], up[6] = {0, 0, 0.35, 0, 0, 0}, down[6] = {0, 0, -0.35, 0, 0, 0};
    ArmJogStep step;
    ArmPose pose;
    memcpy(q, HOME, sizeof q);
    for (int f = 0; f < 720; f += 1) { arm_jog(&k, q, up, 1.0 / 60, &step); memcpy(q, step.q, sizeof q); }
    arm_solve(&k, q, &pose);
    CHECK(pose.trust < 0.01, "the arm should be at the singularity (trust %g)", pose.trust);
    double top = pose.tcp[2];
    for (int f = 0; f < 360; f += 1) {
      arm_jog(&k, q, down, 1.0 / 60, &step);
      memcpy(q, step.q, sizeof q);
      for (int i = 0; i < 6; i += 1) CHECK(q[i] >= GLUON[i].lower && q[i] <= GLUON[i].upper, "joint %d left its limits", i);
    }
    arm_solve(&k, q, &pose);
    CHECK(top - pose.tcp[2] > 0.3, "positionOnly=%d: the tool came down only %.3f m", mode == 0, top - pose.tcp[2]);
  }
}

static void held_keys_stay_bounded(void) {
  for (int mode = 0; mode < 2; mode += 1)
    for (int axis = 0; axis < 6; axis += 1) {
      ArmKinematics k = engine(mode == 0);
      double q[6], twist[6] = {0};
      int flags_or = 0;
      twist[axis >> 1] = axis & 1 ? -0.35 : 0.35;
      memcpy(q, HOME, sizeof q);
      for (int f = 0; f < 3600; f += 1) {
        ArmJogStep step;
        arm_jog(&k, q, twist, 1.0 / 60, &step);
        for (int i = 0; i < 6; i += 1) {
          CHECK(step.q[i] >= GLUON[i].lower && step.q[i] <= GLUON[i].upper, "joint %d left its limits", i);
          CHECK(fabs(step.q[i] - q[i]) <= (3.14 * 0.6) / 60 + 1e-9, "joint %d moved too fast", i);
        }
        memcpy(q, step.q, sizeof q);
        flags_or |= step.flags;
      }
      CHECK(flags_or != 0, "axis %d never reported a limit", axis);
    }
}

static void pinned_arm_backs_out(void) {
  ArmKinematics k = engine(true);
  double q[6], pinned[6], left[6] = {-0.35, 0, 0, 0, 0, 0}, right[6] = {0.35, 0, 0, 0, 0, 0};
  ArmJogStep step;
  memcpy(q, HOME, sizeof q);
  for (int f = 0; f < 900; f += 1) { arm_jog(&k, q, left, 1.0 / 60, &step); memcpy(q, step.q, sizeof q); }
  CHECK(step.limited != 0, "the arm should be pinned at a limit");
  memcpy(pinned, q, sizeof q);
  for (int f = 0; f < 60; f += 1) { arm_jog(&k, q, right, 1.0 / 60, &step); memcpy(q, step.q, sizeof q); }
  double moved = 0;
  for (int i = 0; i < 6; i += 1) moved = fmax(moved, fabs(pinned[i] - q[i]));
  CHECK(moved > 0.05, "moved only %g", moved);
}

static void bad_input_rejected(void) {
  ArmKinematics k = engine(true);
  double q[6], nan6[6], forward[6] = {0.1, 0, 0, 0, 0, 0};
  ArmJogStep step;
  memcpy(q, HOME, sizeof q);
  for (int i = 0; i < 6; i += 1) nan6[i] = NAN;
  CHECK(!arm_jog(&k, q, nan6, 1.0 / 60, &step), "NaN twist accepted");
  CHECK(!arm_jog(&k, q, forward, 0, &step), "dt = 0 accepted");
  q[0] = NAN;
  CHECK(!arm_jog(&k, q, forward, 1.0 / 60, &step), "NaN q accepted");
}

int main(void) {
  struct { const char *name; void (*run)(void); } tests[] = {
    {"FK and trust match the Rust fixture", fk_matches_fixture},
    {"FK matches the independent Python oracle", fk_matches_python_oracle},
    {"Jacobian matches central differences", jacobian_matches_central_differences},
    {"jog checkpoints match the Rust fixture until the stuck-arm escape", jog_matches_fixture},
    {"an arm pushed to full stretch can come back down", stretched_arm_comes_back},
    {"held keys stay inside limits and under the speed cap", held_keys_stay_bounded},
    {"an arm pinned at a joint limit can back out", pinned_arm_backs_out},
    {"non-finite input and dt = 0 are rejected", bad_input_rejected},
  };
  for (size_t i = 0; i < sizeof tests / sizeof tests[0]; i += 1) {
    int before = failures;
    tests[i].run();
    printf("%s  %s\n", failures == before ? "  ok  " : "  FAIL", tests[i].name);
  }
  printf(failures == 0 ? "\nKINEMATICS OK (%d checks)\n" : "\n%d of %d checks failed\n", failures ? failures : checks, checks);
  return failures != 0;
}
