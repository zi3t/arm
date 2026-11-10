// raylib viewer for the GLUON arm, driven by the C kinematics. It mirrors
// ../arm-viewer.js: the same meshes, camera, lighting, jog keys, frames, eased
// tool command and on-arm annotations (joint arcs and labels, BASE and TOOL
// tripods, wrist singularity axes, heading arrow, tool readout). `make viewer`
// builds a desktop window that reads the keyboard itself; `make site` builds
// the /arm/ module, where ../arm-viewer.js owns the keyboard and the HTML
// controls and drives this through the web_* exports.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

#include "build/gluon.h"
#include "kinematics.h"

#if defined(PLATFORM_WEB)
#include <emscripten/emscripten.h>
#include <emscripten/html5.h>
#define GLSL_HEADER "#version 100\nprecision mediump float;\n#define in varying\n#define out_color gl_FragColor\n"
#define GLSL_VS_HEADER "#version 100\n#define in attribute\n#define out varying\n"
#else
#define GLSL_HEADER "#version 330\nout vec4 out_color;\n"
#define GLSL_VS_HEADER "#version 330\n"
#endif

#ifndef MESH_DIR
#define MESH_DIR "meshes"
#endif
#ifndef FONT_FILE
#define FONT_FILE "DepartureMono.ttf"
#endif

// Hemisphere plus key and fill lights with three.js's intensities, Lambert
// diffuse, ACES filmic tone mapping and sRGB output. Monochrome only: the site
// never uses hue.
static const char *VS = GLSL_VS_HEADER
  "in vec3 vertexPosition;\n"
  "in vec3 vertexNormal;\n"
  "uniform mat4 mvp;\n"
  "uniform mat4 matModel;\n"
  "out vec3 normal;\n"
  "void main() {\n"
  "  normal = normalize((matModel * vec4(vertexNormal, 0.0)).xyz);\n"
  "  gl_Position = mvp * vec4(vertexPosition, 1.0);\n"
  "}\n";

static const char *FS = GLSL_HEADER
  "in vec3 normal;\n"
  "uniform float albedo;\n"  // linear
  // three.js ACESFilmicToneMapping; its colour matrices are identity on grey.
  "float aces(float v) {\n"
  "  v /= 0.6;\n"
  "  return clamp((v * (v + 0.0245786) - 0.000090537) / (v * (0.983729 * v + 0.4329510) + 0.238081), 0.0, 1.0);\n"
  "}\n"
  "float srgb(float c) { return c <= 0.0031308 ? c * 12.92 : 1.055 * pow(c, 1.0 / 2.4) - 0.055; }\n"
  "void main() {\n"
  "  vec3 n = normalize(normal);\n"
  "  float sky = mix(0.0908, 1.0, n.y * 0.5 + 0.5) * 1.2;\n"  // HemisphereLight(white, 0x555555, 1.2)
  "  float key = max(dot(n, normalize(vec3(2.0, 3.0, 1.5))), 0.0) * 2.2;\n"
  "  float fill = max(dot(n, normalize(vec3(-2.0, 1.5, -2.0))), 0.0) * 0.6;\n"
  "  float lit = albedo * 0.9 / 3.14159265 * (sky + key + fill);\n"  // Lambert, metalness 0.1
  "  out_color = vec4(vec3(srgb(aces(lit))), 1.0);\n"
  "}\n";

static const char *LINK_FILES[GLUON_JOINTS + 1] = {
  "base_link.glb", "1_Link.glb", "2_Link.glb", "3_Link.glb", "4_Link.glb", "5_Link.glb", "6_Link.glb"
};
static const double HOME[GLUON_JOINTS] = {0, -0.45, 0.95, 0, 0.55, 0};

enum { FRAME_VIEW, FRAME_BASE, FRAME_TOOL };

// Joint annotations, as in arm-viewer.js: a hairline arc around each joint's
// real axis spanning its travel, the last NEAR_LIMIT_RAD before each stop in
// white, a tick at the current angle and a "J3 54°" label. An arc fades in only
// near a stop or while the jog holds that joint back; J shows all six.
#define NEAR_LIMIT_RAD 0.25
#define ARC_STEPS 48
static const float JOINT_ARC_RADIUS[GLUON_JOINTS] = {0.1f, 0.09f, 0.08f, 0.07f, 0.065f, 0.055f};
static const int WRIST_AXES[2] = {3, 5};

// CSS sizes type by em; raylib by ascent-to-descent height, which for
// Departure Mono (ascent 550, descent 150, 550 units per em) is 700/550 em.
#define EM(px) ((px) * 700.0f / 550.0f)

static const Color INK = {0xff, 0xff, 0xff, 0xff};
static const Color DIM = {0xaa, 0xaa, 0xaa, 0xff};

// Shared vertices get the area-weighted sum of their faces' normals, which is
// what three.js computeVertexNormals() does for these normal-less meshes.
static Model load_link(const char *file, Shader shader) {
  char path[512];
  snprintf(path, sizeof path, "%s/%s", MESH_DIR, file);
  Model source = LoadModel(path);
  Mesh merged = {0};
  for (int m = 0; m < source.meshCount; m += 1) {
    merged.vertexCount += source.meshes[m].vertexCount;
    merged.triangleCount += source.meshes[m].triangleCount;
  }
  merged.vertices = MemAlloc(sizeof(float) * 3 * merged.vertexCount);
  merged.normals = MemAlloc(sizeof(float) * 3 * merged.vertexCount);
  merged.indices = MemAlloc(sizeof(unsigned short) * 3 * merged.triangleCount);
  int vbase = 0, ibase = 0;
  for (int m = 0; m < source.meshCount; m += 1) {
    const Mesh *mesh = &source.meshes[m];
    memcpy(merged.vertices + vbase * 3, mesh->vertices, sizeof(float) * 3 * mesh->vertexCount);
    for (int i = 0; i < mesh->triangleCount * 3; i += 1)
      merged.indices[ibase + i] = (unsigned short)(vbase + (mesh->indices ? mesh->indices[i] : i));
    vbase += mesh->vertexCount;
    ibase += mesh->triangleCount * 3;
  }
  const float *v = merged.vertices;
  float *n = merged.normals;
  for (int t = 0; t < merged.triangleCount; t += 1) {
    const unsigned short *tri = &merged.indices[t * 3];
    Vector3 a = {v[tri[0] * 3], v[tri[0] * 3 + 1], v[tri[0] * 3 + 2]};
    Vector3 b = {v[tri[1] * 3], v[tri[1] * 3 + 1], v[tri[1] * 3 + 2]};
    Vector3 c = {v[tri[2] * 3], v[tri[2] * 3 + 1], v[tri[2] * 3 + 2]};
    Vector3 face = Vector3CrossProduct(Vector3Subtract(c, b), Vector3Subtract(a, b));
    for (int k = 0; k < 3; k += 1) {
      n[tri[k] * 3] += face.x;
      n[tri[k] * 3 + 1] += face.y;
      n[tri[k] * 3 + 2] += face.z;
    }
  }
  for (int i = 0; i < merged.vertexCount; i += 1) {
    Vector3 unit = Vector3Normalize((Vector3){n[i * 3], n[i * 3 + 1], n[i * 3 + 2]});
    n[i * 3] = unit.x; n[i * 3 + 1] = unit.y; n[i * 3 + 2] = unit.z;
  }
  UnloadModel(source);
  UploadMesh(&merged, false);
  Model model = LoadModelFromMesh(merged);
  model.materials[0].shader = shader;
  return model;
}

// A rigid frame in the robot's (Z-up) base frame.
typedef struct { Vector3 p; Quaternion q; } Frame3;

typedef struct {
  Vector3 axis, u, v;  // unit axis and the plane its arc lies in
  float radius, alpha;
} JointMark;

// A label queued during the 3D pass and drawn on top afterwards, as the
// three.js sprites and HTML labels sit above the scene.
typedef enum { LABEL_LETTER, LABEL_FRAME, LABEL_JOINT } LabelKind;
typedef struct { LabelKind kind; Vector3 world; char text[16]; float alpha; bool held; } Label;

typedef struct {
  ArmKinematics kin;
  Model links[GLUON_JOINTS + 1];
  Shader shader;
  int albedo_loc;
  Font ui, ui_large, sprite;  // 11px, 22px (both pixel-exact) and a smooth one for 3D labels
  Camera3D camera;
  float yaw, pitch, distance;
  double q[GLUON_JOINTS];
  ArmPose pose;
  Vector3 command;  // eased tool velocity in the robot base frame (m/s)
  Vector3 spin;     // eased tool angular velocity in the robot base frame (rad/s)
  int frame, flags, limited;
  bool reveal_all, reduce_motion, homing;
  Vector3 web_intent;  // [right, forward, up] from the page's held keys
  Vector3 web_turn;    // turns about [right, forward, up]
  bool web_fine, web_fast;
  JointMark marks[GLUON_JOINTS];
  Label labels[32];
  int label_count;
} Viewer;

static Viewer V;

// The URDF is Z-up; raylib, like three.js, is Y-up. Robot (x, y, z) shows at
// world (x, z, -y), as the three.js robot group rotated -90 degrees about X.
static Matrix robot_to_world(void) { return MatrixRotateX(-PI / 2); }
static Vector3 r2w(Vector3 r) { return (Vector3){r.x, r.z, -r.y}; }
static Vector3 world_to_robot(Vector3 w) { return (Vector3){w.x, -w.z, w.y}; }

static Vector3 at_frame(Frame3 f, Vector3 local) { return Vector3Add(f.p, Vector3RotateByQuaternion(local, f.q)); }

static Frame3 link_frame(int joint) {
  const double *l = &V.pose.links[joint * 7];
  return (Frame3){{(float)l[0], (float)l[1], (float)l[2]}, {(float)l[3], (float)l[4], (float)l[5], (float)l[6]}};
}

// The arc's zero reference is the child frame turned back by q, so the arc
// stays put while the tick rides it.
static Frame3 arc_frame(int joint) {
  Frame3 f = link_frame(joint);
  f.q = QuaternionMultiply(f.q, QuaternionFromAxisAngle(V.marks[joint].axis, (float)-V.q[joint]));
  return f;
}

static Vector3 arc_point(int joint, double angle, float scale) {
  const JointMark *m = &V.marks[joint];
  float r = m->radius * scale;
  return Vector3Add(Vector3Scale(m->u, cosf((float)angle) * r), Vector3Scale(m->v, sinf((float)angle) * r));
}

static void queue_label(LabelKind kind, Vector3 robot, const char *text, float alpha, bool held) {
  if (V.label_count >= (int)(sizeof V.labels / sizeof V.labels[0]) || alpha <= 0.01f) return;
  Label *l = &V.labels[V.label_count++];
  *l = (Label){kind, r2w(robot), "", alpha, held};
  snprintf(l->text, sizeof l->text, "%s", text);
}

// A line in robot coordinates; dash > 0 draws it dashed in world units, as
// three.js LineDashedMaterial does.
static void line3(Vector3 a, Vector3 b, Color c, float dash, float gap) {
  a = r2w(a);
  b = r2w(b);
  if (dash <= 0) { DrawLine3D(a, b, c); return; }
  float length = Vector3Distance(a, b);
  Vector3 d = Vector3Normalize(Vector3Subtract(b, a));
  for (float s = 0; s < length; s += dash + gap)
    DrawLine3D(Vector3Add(a, Vector3Scale(d, s)), Vector3Add(a, Vector3Scale(d, fminf(s + dash, length))), c);
}

static void init_marks(void) {
  for (int i = 0; i < GLUON_JOINTS; i += 1) {
    JointMark *m = &V.marks[i];
    m->axis = Vector3Normalize((Vector3){(float)GLUON[i].axis[0], (float)GLUON[i].axis[1], (float)GLUON[i].axis[2]});
    Vector3 helper = fabsf(m->axis.x) < 0.9f ? (Vector3){1, 0, 0} : (Vector3){0, 1, 0};
    m->u = Vector3Normalize(Vector3CrossProduct(m->axis, helper));
    m->v = Vector3CrossProduct(m->axis, m->u);
    m->radius = JOINT_ARC_RADIUS[i];
  }
}

static void update_camera(void) {
  // Narrow stages keep the whole arm in frame instead of cropping it (three.js
  // camera.zoom = min(1, aspect / 1.25) on a 42 degree fov).
  float aspect = (float)GetScreenWidth() / (float)GetScreenHeight();
  float zoom = fminf(1, aspect / 1.25f);
  V.camera.fovy = 2 * atanf(tanf(21 * DEG2RAD) / zoom) * RAD2DEG;
  V.camera.position = (Vector3){
    V.camera.target.x + V.distance * cosf(V.pitch) * sinf(V.yaw),
    V.camera.target.y + V.distance * sinf(V.pitch),
    V.camera.target.z + V.distance * cosf(V.pitch) * cosf(V.yaw),
  };
}

// [right, forward, up] intent as a vector in the robot base frame.
static Vector3 intent_to_robot(Vector3 intent) {
  if (V.frame == FRAME_BASE) return intent;
  if (V.frame == FRAME_TOOL) return Vector3RotateByQuaternion(intent, link_frame(GLUON_JOINTS - 1).q);
  // View frame: "forward" is the way the camera looks, flattened onto the floor.
  Vector3 up = {0, 1, 0}, forward = Vector3Subtract(V.camera.target, V.camera.position);
  forward.y = 0;
  forward = Vector3Normalize(forward);
  Vector3 right = Vector3Normalize(Vector3CrossProduct(forward, up));
  Vector3 world = Vector3Add(Vector3Add(Vector3Scale(right, intent.x), Vector3Scale(forward, intent.y)), Vector3Scale(up, intent.z));
  return world_to_robot(world);
}

// Home drives the arm back instead of jumping: every joint moves along a
// straight line in joint space so they arrive together, the slowest at its
// jog speed cap, easing in over the last stretch. Any jog key cancels it.
#define HOME_SPEED_FRACTION 0.6
#define HOME_EASE_SECONDS 0.3
static void start_home(void) {
  V.homing = true;
  V.command = V.spin = (Vector3){0, 0, 0};
}

static void step_home(float dt) {
  double remaining = 0, largest = 0;
  for (int i = 0; i < GLUON_JOINTS; i += 1) {
    double d = fabs(HOME[i] - V.q[i]);
    remaining = fmax(remaining, d / (GLUON[i].velocity * HOME_SPEED_FRACTION));
    largest = fmax(largest, d);
  }
  if (largest < 1e-4) {
    memcpy(V.q, HOME, sizeof V.q);
    V.homing = false;
    return;
  }
  double share = fmin(1, dt / fmax(remaining, HOME_EASE_SECONDS));
  for (int i = 0; i < GLUON_JOINTS; i += 1) V.q[i] += (HOME[i] - V.q[i]) * share;
}

#if !defined(PLATFORM_WEB)
static void handle_keys(void) {
  if (IsKeyPressed(KEY_V)) V.frame = FRAME_VIEW;
  if (IsKeyPressed(KEY_B)) V.frame = FRAME_BASE;
  if (IsKeyPressed(KEY_T)) V.frame = FRAME_TOOL;
  if (IsKeyPressed(KEY_X)) V.reveal_all = !V.reveal_all;
  if (IsKeyPressed(KEY_H)) start_home();
}
#endif

#if !defined(PLATFORM_WEB)
static Vector3 held_intent(void) {
  Vector3 intent = {0, 0, 0};
  if (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT)) intent.x -= 1;
  if (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT)) intent.x += 1;
  if (IsKeyDown(KEY_W)) intent.y += 1;
  if (IsKeyDown(KEY_S)) intent.y -= 1;
  if (IsKeyDown(KEY_R) || IsKeyDown(KEY_UP)) intent.z += 1;
  if (IsKeyDown(KEY_F) || IsKeyDown(KEY_DOWN)) intent.z -= 1;
  return intent;
}

// I K tip the tool down and up (about "right"), J L turn it left and right
// (about "up"), U O roll it left and right (about "forward").
static Vector3 held_turn(void) {
  Vector3 turn = {0, 0, 0};
  if (IsKeyDown(KEY_I)) turn.x -= 1;
  if (IsKeyDown(KEY_K)) turn.x += 1;
  if (IsKeyDown(KEY_J)) turn.z += 1;
  if (IsKeyDown(KEY_L)) turn.z -= 1;
  if (IsKeyDown(KEY_U)) turn.y -= 1;
  if (IsKeyDown(KEY_O)) turn.y += 1;
  return turn;
}
#endif

// A held direction, capped at unit length, scaled to a speed and turned from
// the chosen frame into the robot base frame.
static Vector3 command_target(Vector3 intent, float speed) {
  if (Vector3LengthSqr(intent) == 0) return (Vector3){0, 0, 0};
  if (Vector3Length(intent) > 1) intent = Vector3Normalize(intent);
  return intent_to_robot(Vector3Scale(intent, speed));
}

static Vector3 ease(Vector3 command, Vector3 target, float share) {
  command = Vector3Lerp(command, target, share);
  return Vector3LengthSqr(command) < 1e-8f && Vector3LengthSqr(target) == 0 ? (Vector3){0, 0, 0} : command;
}

// One frame of jogging. The commanded tool velocity and turn rate ease toward
// the held keys so starting, stopping and reversing do not kick the joints.
// Shift is fine, Space is fast. Moving alone constrains only the tool tip, so
// the wrist follows freely; while turning, the solver holds the full pose, so
// the tool turns about its tip.
static void step(float dt, Vector3 intent, Vector3 turn, bool fine, bool fast) {
  const float JOG_TIME_CONSTANT = 0.07f;
  Vector3 target = command_target(intent, fine ? 0.04f : fast ? 0.35f : 0.15f);
  Vector3 spin_target = command_target(turn, fine ? 0.15f : fast ? 1.0f : 0.5f);
  if (V.homing && (Vector3LengthSqr(target) > 0 || Vector3LengthSqr(spin_target) > 0)) V.homing = false;
  if (V.homing) {
    step_home(dt);
    V.flags = V.limited = 0;
    arm_solve(&V.kin, V.q, &V.pose);
    goto fade;
  }
  float share = 1 - expf(-dt / JOG_TIME_CONSTANT);
  V.command = ease(V.command, target, share);
  V.spin = ease(V.spin, spin_target, share);
  V.kin.position_only = Vector3LengthSqr(V.spin) == 0;
  V.flags = V.limited = 0;
  if (Vector3LengthSqr(V.command) > 0 || Vector3LengthSqr(V.spin) > 0) {
    double twist[6] = {V.command.x, V.command.y, V.command.z, V.spin.x, V.spin.y, V.spin.z};
    ArmJogStep out;
    if (arm_jog(&V.kin, V.q, twist, dt, &out)) {
      memcpy(V.q, out.q, sizeof V.q);
      V.flags = out.flags;
      V.limited = out.limited;
    } else {
      V.command = V.spin = (Vector3){0, 0, 0};
    }
  }
  arm_solve(&V.kin, V.q, &V.pose);

fade:
  for (int i = 0; i < GLUON_JOINTS; i += 1) {
    double margin = fmin(V.q[i] - GLUON[i].lower, GLUON[i].upper - V.q[i]);
    bool held = (V.limited >> i) & 1, near = margin < NEAR_LIMIT_RAD;
    float target_alpha = held || near || V.reveal_all ? 1 : 0, *alpha = &V.marks[i].alpha;
    *alpha += (target_alpha - *alpha) * (1 - expf(-fmaxf(0, dt) / 0.12f));
    if (V.reduce_motion || fabsf(target_alpha - *alpha) < 0.01f) *alpha = target_alpha;
  }
}

static void draw_grid(void) {
  // three.js GridHelper(2, 20, 0x555555, 0x2a2a2a) at 0.8 opacity.
  for (int i = -10; i <= 10; i += 1) {
    float s = i * 0.1f;
    Color c = i == 0 ? (Color){0x44, 0x44, 0x44, 255} : (Color){0x22, 0x22, 0x22, 255};
    DrawLine3D((Vector3){s, 0, -1}, (Vector3){s, 0, 1}, c);
    DrawLine3D((Vector3){-1, 0, s}, (Vector3){1, 0, s}, c);
  }
}

// Frame axes are quiet reference lines, told apart by line style rather than
// colour: x solid, y dashed, z dotted, each lettered, with the frame's name.
static void draw_tripod(Frame3 f, float length, const char *name) {
  static const float DASH[3][2] = {{0, 0}, {0.012f, 0.01f}, {0.004f, 0.008f}};
  static const char *LETTERS[3] = {"X", "Y", "Z"};
  for (int a = 0; a < 3; a += 1) {
    Vector3 end = {a == 0 ? length : 0, a == 1 ? length : 0, a == 2 ? length : 0};
    line3(f.p, at_frame(f, end), ColorAlpha(INK, 0.8f), DASH[a][0], DASH[a][1]);
    queue_label(LABEL_LETTER, at_frame(f, Vector3Scale(end, 1.14f)), LETTERS[a], 0.85f, false);
  }
  queue_label(LABEL_FRAME, at_frame(f, (Vector3){0, 0, length * -0.3f}), name, 1, false);
}

static void draw_joint_marks(void) {
  float proximity = 1 - Clamp((float)V.pose.trust, 0, 1);
  for (int i = 0; i < GLUON_JOINTS; i += 1) {
    const JointMark *m = &V.marks[i];
    Frame3 f = arc_frame(i);
    double lower = GLUON[i].lower, upper = GLUON[i].upper, q = V.q[i];
    double near = fmin(NEAR_LIMIT_RAD, (upper - lower) / 2);
    bool held = (V.limited >> i) & 1, close = fmin(q - lower, upper - q) < NEAR_LIMIT_RAD;

    for (int w = 0; w < 2; w += 1) {
      if (WRIST_AXES[w] != i || proximity <= 0.05f) continue;
      Vector3 reach = Vector3Scale(m->axis, 0.16f);
      line3(at_frame(f, Vector3Negate(reach)), at_frame(f, reach), ColorAlpha(INK, 0.25f + 0.75f * proximity), 0.012f, 0.01f);
    }
    if (m->alpha <= 0.01f) continue;

    Color track = ColorAlpha(held || close ? INK : DIM, m->alpha * (held || close ? 0.95f : 0.7f));
    Color stop = ColorAlpha(INK, m->alpha);
    for (int s = 0; s < ARC_STEPS; s += 1) {
      double a = lower + (upper - lower) * s / ARC_STEPS, b = lower + (upper - lower) * (s + 1) / ARC_STEPS;
      line3(at_frame(f, arc_point(i, a, 1)), at_frame(f, arc_point(i, b, 1)), track, 0, 0);
    }
    for (int s = 0; s < 8; s += 1) {
      double a0 = lower + near * s / 8, a1 = lower + near * (s + 1) / 8;
      double b0 = upper - near + near * s / 8, b1 = upper - near + near * (s + 1) / 8;
      line3(at_frame(f, arc_point(i, a0, 1)), at_frame(f, arc_point(i, a1, 1)), stop, 0, 0);
      line3(at_frame(f, arc_point(i, b0, 1)), at_frame(f, arc_point(i, b1, 1)), stop, 0, 0);
    }
    line3(at_frame(f, arc_point(i, lower, 0.88f)), at_frame(f, arc_point(i, lower, 1.12f)), track, 0, 0);
    line3(at_frame(f, arc_point(i, upper, 0.88f)), at_frame(f, arc_point(i, upper, 1.12f)), track, 0, 0);
    line3(at_frame(f, arc_point(i, q, 0.8f)), at_frame(f, arc_point(i, q, 1.14f)), stop, 0, 0);

    char text[16];
    snprintf(text, sizeof text, "J%d %ld\xC2\xB0", i + 1, lround(q * 180 / PI));  // never "-0"
    queue_label(LABEL_JOINT, at_frame(f, arc_point(i, q, 1.34f)), text, m->alpha, held);
  }
}

static void draw_heading(void) {
  float speed = Vector3Length(V.command);
  if (speed * speed <= 1e-6f) return;
  // three.js ArrowHelper: length 0.03 + |command| * 0.5, head 0.03 long, 0.02 wide.
  Vector3 tcp = {(float)V.pose.tcp[0], (float)V.pose.tcp[1], (float)V.pose.tcp[2]};
  Vector3 dir = Vector3Normalize(V.command);
  float length = 0.03f + speed * 0.5f;
  Vector3 neck = Vector3Add(tcp, Vector3Scale(dir, length - 0.03f)), tip = Vector3Add(tcp, Vector3Scale(dir, length));
  DrawLine3D(r2w(tcp), r2w(neck), INK);
  DrawCylinderEx(r2w(neck), r2w(tip), 0.01f, 0, 12, INK);
}

// Pixels per world unit at a point, for labels that keep a world size.
static float pixels_per_unit(Vector3 world) {
  Vector3 forward = Vector3Normalize(Vector3Subtract(V.camera.target, V.camera.position));
  float depth = Vector3DotProduct(Vector3Subtract(world, V.camera.position), forward);
  return depth <= 0 ? 0 : GetScreenHeight() / (2 * tanf(V.camera.fovy * DEG2RAD / 2) * depth);
}

static void draw_labels(void) {
  for (int i = 0; i < V.label_count; i += 1) {
    const Label *l = &V.labels[i];
    float scale = pixels_per_unit(l->world);
    if (scale <= 0) continue;
    Vector2 at = GetWorldToScreen(l->world, V.camera);
    if (l->kind == LABEL_LETTER) {
      // A 52px glyph on a 96px sprite 0.022 units wide.
      float size = EM(0.022f * 52 / 96 * scale);
      Vector2 extent = MeasureTextEx(V.sprite, l->text, size, 0);
      DrawTextEx(V.sprite, l->text, (Vector2){at.x - extent.x / 2, at.y - extent.y / 2}, size, 0, ColorAlpha(INK, l->alpha));
    } else if (l->kind == LABEL_FRAME) {
      // A 28px lowercase name, 4px in, on a 256 x 64 sprite 0.16 x 0.04 units.
      float size = EM(0.04f * 28 / 64 * scale);
      DrawTextEx(V.sprite, l->text, (Vector2){at.x + 0.16f * 4 / 256 * scale, at.y - size / 2}, size, 0, DIM);
    } else {
      // The HTML .joint-label: 11px type on an 11px-high box (line-height 1),
      // nudged right 0.55rem and up half an em, black on white when held.
      Vector2 extent = MeasureTextEx(V.ui, l->text, EM(11), 0);
      Rectangle box = {roundf(at.x + 8.8f), roundf(at.y - 5.5f), extent.x + 8, 11};
      DrawRectangleRec(box, ColorAlpha(l->held ? INK : BLACK, l->alpha));
      DrawTextEx(V.ui, l->text, (Vector2){box.x + 4, box.y - 1.5f}, EM(11), 0, ColorAlpha(l->held ? BLACK : INK, l->alpha));
    }
  }
}

#if !defined(PLATFORM_WEB)
// The .arm-tcp readout: x, y, z down the top left in 22px type, letters grey,
// values right-aligned with a true minus sign.
static void draw_readout(void) {
  float inset = Clamp(GetScreenWidth() * 0.024f, 16, 30.4f);
  static const char *AXES[3] = {"x", "y", "z"};
  for (int a = 0; a < 3; a += 1) {
    float y = inset + a * (33 + 2.4f) + (33 - EM(22)) / 2;  // 1.5 line-height
    char value[24];
    double v = V.pose.tcp[a];
    snprintf(value, sizeof value, "%s%.3f", v < 0 ? "\xE2\x88\x92" : "", fabs(v));
    DrawTextEx(V.ui_large, AXES[a], (Vector2){inset, y}, EM(22), 0, DIM);
    float width = MeasureTextEx(V.ui_large, value, EM(22), 0).x;
    DrawTextEx(V.ui_large, value, (Vector2){inset + 32 + 136 - width, y}, EM(22), 0, INK);
  }
}
#endif

static void draw(void) {
  V.label_count = 0;
  BeginDrawing();
  ClearBackground(BLACK);
  BeginMode3D(V.camera);
  draw_grid();
  Matrix up = robot_to_world();
  for (int i = 0; i <= GLUON_JOINTS; i += 1) {
    Matrix place = MatrixIdentity();
    if (i > 0) {
      Frame3 f = link_frame(i - 1);
      place = MatrixMultiply(QuaternionToMatrix(f.q), MatrixTranslate(f.p.x, f.p.y, f.p.z));
    }
    V.links[i].transform = MatrixMultiply(place, up);
    float albedo = i == GLUON_JOINTS ? 1.0f : 0.479f;  // 0xb8b8b8 in linear; the tool link is white
    SetShaderValue(V.shader, V.albedo_loc, &albedo, SHADER_UNIFORM_FLOAT);
    DrawModel(V.links[i], (Vector3){0, 0, 0}, 1, WHITE);
  }
  draw_tripod((Frame3){{0, 0, 0}, QuaternionIdentity()}, 0.16f, "base");
  draw_tripod(link_frame(GLUON_JOINTS - 1), 0.11f, "tool");
  draw_heading();
  // Joint marks draw over the arm, like the three.js lines with depthTest off.
  rlDrawRenderBatchActive();
  rlDisableDepthTest();
  draw_joint_marks();
  rlDrawRenderBatchActive();
  rlEnableDepthTest();
  EndMode3D();
  draw_labels();
#if !defined(PLATFORM_WEB)
  draw_readout();  // on the page, the HTML .arm-tcp readout shows these
#endif
  EndDrawing();
}

static void frame(void) {
  float dt = fminf(0.05f, GetFrameTime());
#if defined(PLATFORM_WEB)
  // The page sizes the canvas; render at that size, not stretched.
  double css_w, css_h;
  emscripten_get_element_css_size("#arm-canvas", &css_w, &css_h);
  if ((int)css_w != GetScreenWidth() || (int)css_h != GetScreenHeight()) SetWindowSize((int)css_w, (int)css_h);
#else
  handle_keys();
#endif
  if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
    Vector2 d = GetMouseDelta();
    V.yaw -= d.x * 0.008f;
    V.pitch = Clamp(V.pitch + d.y * 0.008f, -0.2f, 1.45f);
  }
  V.distance = Clamp(V.distance * (1 - GetMouseWheelMove() * 0.1f), 0.3f, 3.0f);
  update_camera();
#if defined(PLATFORM_WEB)
  step(dt, V.web_intent, V.web_turn, V.web_fine, V.web_fast);
#else
  step(dt, held_intent(), held_turn(), IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT), IsKeyDown(KEY_SPACE));
#endif
  draw();
}

#if defined(PLATFORM_WEB)
// The page's controls. Keys stay in JavaScript so they work only while the
// arm has focus and never reach GLFW, which would cancel Tab page-wide.
EMSCRIPTEN_KEEPALIVE void web_input(double right, double forward, double up,
                                    double turn_right, double turn_forward, double turn_up, int fine, int fast) {
  V.web_intent = (Vector3){(float)right, (float)forward, (float)up};
  V.web_turn = (Vector3){(float)turn_right, (float)turn_forward, (float)turn_up};
  V.web_fine = fine;
  V.web_fast = fast;
}
EMSCRIPTEN_KEEPALIVE void web_set_frame(int frame) { V.frame = frame; }
EMSCRIPTEN_KEEPALIVE void web_set_joints(int on) { V.reveal_all = on; }
EMSCRIPTEN_KEEPALIVE void web_set_reduce_motion(int on) { V.reduce_motion = on; }
EMSCRIPTEN_KEEPALIVE void web_home(void) { start_home(); }

// What the page reads each frame: tcp[3], q[6], flags, limited, trust, a mask
// of joints within NEAR_LIMIT_RAD of a stop, and whether it is driving home.
EMSCRIPTEN_KEEPALIVE double *web_state(void) {
  static double state[3 + GLUON_JOINTS + 5];
  int near = 0;
  for (int i = 0; i < GLUON_JOINTS; i += 1) {
    if (fmin(V.q[i] - GLUON[i].lower, GLUON[i].upper - V.q[i]) < NEAR_LIMIT_RAD) near |= 1 << i;
    state[3 + i] = V.q[i];
  }
  memcpy(state, V.pose.tcp, sizeof V.pose.tcp);
  state[3 + GLUON_JOINTS] = V.flags;
  state[4 + GLUON_JOINTS] = V.limited;
  state[5 + GLUON_JOINTS] = V.pose.trust;
  state[6 + GLUON_JOINTS] = near;
  state[7 + GLUON_JOINTS] = V.homing;
  return state;
}
#endif

static Font load_font(int size, int filter) {
  // Printable ASCII plus the degree sign and a true minus.
  int codepoints[97];
  for (int c = 0; c < 95; c += 1) codepoints[c] = 32 + c;
  codepoints[95] = 0xB0;
  codepoints[96] = 0x2212;
  Font font = LoadFontEx(FONT_FILE, size, codepoints, 97);
  SetTextureFilter(font.texture, filter);
  return font;
}

int main(int argc, char **argv) {
  // --shot out.png [--jog x,y,z] [--turn x,y,z] [--frames n] [--joints 1]
  // renders headless for checks.
  const char *shot = NULL;
  Vector3 scripted = {0, 0, 0}, scripted_turn = {0, 0, 0};
  int frames = 1;
  for (int i = 1; i + 1 < argc; i += 2) {
    if (!strcmp(argv[i], "--shot")) shot = argv[i + 1];
    else if (!strcmp(argv[i], "--frames")) frames = atoi(argv[i + 1]);
    else if (!strcmp(argv[i], "--jog")) sscanf(argv[i + 1], "%f,%f,%f", &scripted.x, &scripted.y, &scripted.z);
    else if (!strcmp(argv[i], "--turn")) sscanf(argv[i + 1], "%f,%f,%f", &scripted_turn.x, &scripted_turn.y, &scripted_turn.z);
    else if (!strcmp(argv[i], "--joints")) V.reveal_all = atoi(argv[i + 1]) != 0;
  }

#if defined(PLATFORM_WEB)
  SetTraceLogLevel(LOG_WARNING);
#endif
  SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE | (shot ? FLAG_WINDOW_HIDDEN : 0));
  InitWindow(960, 600, "arm");
  arm_init(&V.kin, GLUON, GLUON_JOINTS, HOME);
  memcpy(V.q, HOME, sizeof V.q);
  init_marks();
  V.shader = LoadShaderFromMemory(VS, FS);
  V.albedo_loc = GetShaderLocation(V.shader, "albedo");
  for (int i = 0; i <= GLUON_JOINTS; i += 1) V.links[i] = load_link(LINK_FILES[i], V.shader);
  V.ui = load_font((int)EM(11), TEXTURE_FILTER_POINT);
  V.ui_large = load_font((int)EM(22), TEXTURE_FILTER_POINT);
  V.sprite = load_font((int)EM(52), TEXTURE_FILTER_BILINEAR);
  // Camera at (0.68, 0.46, 0.68) looking at (0, 0.25, 0), as in three.js.
  V.camera = (Camera3D){.target = {0, 0.25f, 0}, .up = {0, 1, 0}, .fovy = 42, .projection = CAMERA_PERSPECTIVE};
  V.yaw = PI / 4;
  V.distance = sqrtf(0.68f * 0.68f * 2 + 0.21f * 0.21f);
  V.pitch = asinf(0.21f / V.distance);
  update_camera();
  arm_solve(&V.kin, V.q, &V.pose);

  if (shot) {
    for (int f = 0; f < frames; f += 1) {
      update_camera();
      step(1.0f / 60, scripted, scripted_turn, false, false);
      draw();
    }
    TakeScreenshot(shot);
    const double *tool = &V.pose.links[(GLUON_JOINTS - 1) * 7 + 3];
    printf("tcp %.4f %.4f %.4f quat %.3f %.3f %.3f %.3f flags %d limited %d\n", V.pose.tcp[0], V.pose.tcp[1], V.pose.tcp[2],
           tool[0], tool[1], tool[2], tool[3], V.flags, V.limited);
  } else {
#if defined(PLATFORM_WEB)
    emscripten_set_main_loop(frame, 0, 1);
#else
    SetTargetFPS(60);
    while (!WindowShouldClose()) frame();
#endif
  }

  for (int i = 0; i <= GLUON_JOINTS; i += 1) UnloadModel(V.links[i]);
  UnloadFont(V.ui);
  UnloadFont(V.ui_large);
  UnloadFont(V.sprite);
  UnloadShader(V.shader);
  CloseWindow();
  return 0;
}
