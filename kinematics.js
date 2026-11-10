// Ported from the author's MIT-licensed gluon-kinematics Rust crate, retired at tag gluon-rust-final.

// Forward kinematics and bounded Cartesian jogging for a serial arm of revolute
// joints. Each joint is { origin: [x, y, z], axis: [x, y, z], lower, upper,
// velocity }, with origin rpy zero. Loop order follows the Rust original so the
// results match its recorded fixtures (kinematics.fixture.json) to round-off,
// except for the stuck-arm escape (escapeDrive), which the Rust build lacked.
const STOP_SIGMA = 0.004;
const WARN_SIGMA = 0.04;
const BASE_DAMPING = 0.01;
const MAX_DAMPING = 0.15;
const SPEED_FRACTION = 0.6;
const LIMIT_RATE = 8;
const ESCAPE_RATE = 1;
const ESCAPE_FULL_SPEED = 0.05;
export const JOG_NEAR_SINGULARITY = 1;
export const JOG_JOINT_LIMIT = 2;
export const JOG_SPEED_LIMITED = 4;
export const JOG_ESCAPING = 8;

const mul = (a, b) => a.map((row) => [0, 1, 2].map((c) => row[0] * b[0][c] + row[1] * b[1][c] + row[2] * b[2][c]));
const apply = (m, v) => m.map((row) => row[0] * v[0] + row[1] * v[1] + row[2] * v[2]);

// Rodrigues rotation about a (normalised) axis.
function rotation(axis, angle) {
  const n = Math.hypot(...axis);
  const [x, y, z] = axis.map((v) => v / n);
  const c = Math.cos(angle), s = Math.sin(angle), t = 1 - c;
  return [
    [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
    [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
    [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
  ];
}

function toQuaternion([[a, b, c], [d, e, f], [g, h, i]]) {
  const trace = a + e + i;
  if (trace > 0) { const s = Math.sqrt(trace + 1) * 2; return [(h - f) / s, (c - g) / s, (d - b) / s, 0.25 * s]; }
  if (a > e && a > i) { const s = Math.sqrt(1 + a - e - i) * 2; return [0.25 * s, (b + d) / s, (c + g) / s, (h - f) / s]; }
  if (e > i) { const s = Math.sqrt(1 + e - a - i) * 2; return [(b + d) / s, 0.25 * s, (f + h) / s, (c - g) / s]; }
  const s = Math.sqrt(1 + i - a - e) * 2;
  return [(c + g) / s, (f + h) / s, 0.25 * s, (d - b) / s];
}

// Per joint: its pivot p, its world axis z before it turns, and the frame R after.
function frames(joints, q) {
  let R = [[1, 0, 0], [0, 1, 0], [0, 0, 1]], p = [0, 0, 0];
  return joints.map((joint, i) => {
    p = apply(R, joint.origin).map((v, k) => v + p[k]);
    const z = apply(R, joint.axis), zn = Math.hypot(...z);
    R = mul(R, rotation(joint.axis, q[i]));
    return { p, z: z.map((v) => v / zn), R };
  });
}

// 6 x n geometric Jacobian of the tool point (the last joint's origin).
export function jacobian(joints, q) {
  const f = frames(joints, q), tcp = f.at(-1).p;
  const J = Array.from({ length: 6 }, () => Array(joints.length).fill(0));
  f.forEach(({ p, z }, i) => {
    const d = [tcp[0] - p[0], tcp[1] - p[1], tcp[2] - p[2]];
    const linear = [z[1] * d[2] - z[2] * d[1], z[2] * d[0] - z[0] * d[2], z[0] * d[1] - z[1] * d[0]];
    for (let r = 0; r < 3; r += 1) { J[r][i] = linear[r]; J[r + 3][i] = z[r]; }
  });
  return J;
}

const smoothstep = (sigma) => {
  const x = Math.min(1, Math.max(0, (sigma - STOP_SIGMA) / (WARN_SIGMA - STOP_SIGMA)));
  return Number.isFinite(sigma) ? x * x * (3 - 2 * x) : 0;
};

// Cyclic Jacobi eigendecomposition of a symmetric matrix; vecs[k][i] is
// component k of eigenvector i.
function eigen(input) {
  const n = input.length, a = input.map((row) => row.slice());
  const vecs = a.map((_, i) => a.map((__, j) => (i === j ? 1 : 0)));
  for (let sweep = 0; sweep < 100; sweep += 1) {
    let off = 0;
    for (let p = 0; p < n; p += 1) for (let q = p + 1; q < n; q += 1) off += a[p][q] * a[p][q];
    if (off <= 1e-30) return { vecs, vals: a.map((row, i) => row[i]) };
    for (let p = 0; p < n; p += 1) for (let q = p + 1; q < n; q += 1) {
      const apq = a[p][q];
      if (Math.abs(apq) <= 1e-300) continue;
      const theta = (a[q][q] - a[p][p]) / (2 * apq);
      const t = (theta >= 0 ? 1 : -1) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
      const c = 1 / Math.sqrt(t * t + 1), s = t * c;
      for (let k = 0; k < n; k += 1) { const kp = a[k][p], kq = a[k][q]; a[k][p] = c * kp - s * kq; a[k][q] = s * kp + c * kq; }
      for (let k = 0; k < n; k += 1) { const pk = a[p][k], qk = a[q][k]; a[p][k] = c * pk - s * qk; a[q][k] = s * pk + c * qk; }
      for (const row of vecs) { const vp = row[p], vq = row[q]; row[p] = c * vp - s * vq; row[q] = s * vp + c * vq; }
    }
  }
  throw new Error("Jacobi eigendecomposition did not converge");
}

// Damped least squares with locked joints held at `fixed` rates. sigma is the
// smallest singular value among the free task directions.
function solve(jac, locked, fixed, twist, positionOnly) {
  const n = locked.length, rows = positionOnly ? 3 : 6;
  const j = jac.map((row, r) => row.map((cell, k) => (locked[k] || r >= rows ? 0 : cell)));
  const residual = twist.slice();
  for (let r = 0; r < rows; r += 1) for (let k = 0; k < n; k += 1) if (locked[k]) residual[r] -= jac[r][k] * fixed[k];
  const S = j[0].map((_, a) => j[0].map((__, b) => j.reduce((sum, row) => sum + row[a] * row[b], 0)));
  const { vecs, vals } = eigen(S);
  const lockedAxis = (i) => locked.some((l, k) => l && Math.abs(vecs[k][i]) > 0.5);
  const ranked = [];
  for (let i = 0; i < n; i += 1) if (!lockedAxis(i)) ranked.push(Math.sqrt(Math.max(vals[i], 0)));
  const rank = Math.min(ranked.length, rows);
  if (rank === 0) return { dq: fixed.slice(), sigma: 0 };
  ranked.sort((x, y) => y - x);
  const sigma = ranked[rank - 1];
  const untrusted = 1 - smoothstep(sigma);
  const lambda = BASE_DAMPING + (MAX_DAMPING - BASE_DAMPING) * untrusted * untrusted;
  const dq = fixed.slice();
  for (let i = 0; i < n; i += 1) {
    if (lockedAxis(i)) continue;
    let numerator = 0;
    for (let r = 0; r < 6; r += 1) {
      let projected = 0;
      for (let c = 0; c < n; c += 1) projected += j[r][c] * vecs[c][i];
      numerator += projected * residual[r];
    }
    const coefficient = numerator / (Math.max(vals[i], 0) + lambda * lambda);
    for (let r = 0; r < n; r += 1) dq[r] += coefficient * vecs[r][i];
  }
  return { dq, sigma };
}

// A fully stretched arm cannot move along itself at first order, in either
// direction, so damped least squares alone leaves it stuck there. When the
// position Jacobian is singular along the shoulder-to-tool line and the command
// points back toward the shoulder, return a joint rate that nudges the arm
// toward its rest pose; once it bends, the normal solver follows the command.
function escapeDrive(joints, q, jac, twist) {
  const f = frames(joints, q), tcp = f.at(-1).p, shoulder = f[1].p;
  const radial = tcp.map((v, k) => v - shoulder[k]), length = Math.hypot(...radial);
  if (length < 1e-9) return 0;
  const unit = radial.map((v) => v / length);
  const gram = [0, 1, 2].map((a) => [0, 1, 2].map((b) => jac[a].reduce((sum, v, k) => sum + v * jac[b][k], 0)));
  const { vecs, vals } = eigen(gram);
  const weakest = vals.indexOf(Math.min(...vals));
  const sigma = Math.sqrt(Math.max(vals[weakest], 0));
  const along = Math.abs(vecs[0][weakest] * unit[0] + vecs[1][weakest] * unit[1] + vecs[2][weakest] * unit[2]);
  const inward = -(twist[0] * unit[0] + twist[1] * unit[1] + twist[2] * unit[2]);
  // Only a command that is mostly back toward the shoulder asks to leave.
  if (sigma >= WARN_SIGMA || along < 0.7 || inward < 0.5 * Math.hypot(twist[0], twist[1], twist[2])) return 0;
  return ESCAPE_RATE * (1 - smoothstep(sigma)) * Math.min(1, inward / ESCAPE_FULL_SPEED);
}

export function createKinematics(joints, rest = Array(joints.length).fill(0)) {
  const n = joints.length;
  const clamp = (v, i) => Math.min(joints[i].upper, Math.max(joints[i].lower, v));
  // Free orientation (the default) constrains only the tool position, which
  // stalls far less often than also holding the wrist orientation.
  let positionOnly = true;
  return {
    setPositionOnly(on) { positionOnly = Boolean(on); },
    // Link frames as [px, py, pz, qx, qy, qz, qw] per joint, the tool point,
    // and how far the current mode is from a singularity (1 clear, 0 at one).
    solve(q) {
      const f = frames(joints, q);
      const links = new Float64Array(n * 7);
      f.forEach(({ p, R }, i) => links.set([...p, ...toQuaternion(R)], i * 7));
      const none = Array(n).fill(false), zeros = Array(n).fill(0);
      const { sigma } = solve(jacobian(joints, q), none, zeros, [0, 0, 0, 0, 0, 0], positionOnly);
      return { links, tcp: Float64Array.from(f.at(-1).p), trust: smoothstep(sigma) };
    },
    // One bounded jog step toward `twist` ([vx vy vz wx wy wz]) over dt. It
    // keeps every joint inside its limits, damps motion near a singularity and
    // caps joint speed, so a held key never snaps the arm between
    // configurations. Returns null on non-finite input.
    jog(q, twist, dt) {
      if (!(dt > 0) || !Number.isFinite(dt) || ![...q, ...twist].every(Number.isFinite)) return null;
      const current = Array.from(q, clamp);
      const jac = jacobian(joints, current);
      const locked = Array(n).fill(false), fixed = Array(n).fill(0);
      const drive = escapeDrive(joints, current, jac, twist);
      const away = current.map((v, i) => rest[i] - v), awayLength = Math.hypot(...away);
      let dq, sigma, speedLimited = false;
      for (let pass = 0; pass < 7; pass += 1) {
        const solved = solve(jac, locked, fixed, twist, positionOnly);
        if (drive > 0 && awayLength > 1e-9) solved.dq.forEach((_, i) => { if (!locked[i]) solved.dq[i] += (drive * away[i]) / awayLength; });
        let shrink = 1;
        solved.dq.forEach((rate, i) => {
          const cap = joints[i].velocity * SPEED_FRACTION;
          if (Math.abs(rate) > cap) shrink = Math.min(shrink, cap / Math.abs(rate));
        });
        speedLimited = shrink < 1;
        const rates = speedLimited ? solved.dq.map((rate) => rate * shrink) : solved.dq;
        let newlyLocked = false;
        for (let i = 0; i < n; i += 1) {
          if (locked[i] || rates[i] === 0) continue;
          const outward = rates[i] > 0;
          const distance = outward ? joints[i].upper - current[i] : current[i] - joints[i].lower;
          const cap = Math.max(0, Math.min(LIMIT_RATE * distance, distance / dt));
          if (Math.abs(rates[i]) > cap) { locked[i] = true; fixed[i] = outward ? cap : -cap; newlyLocked = true; }
        }
        dq = rates;
        sigma = solved.sigma;
        if (!newlyLocked) break;
      }
      const limited = locked.reduce((mask, l, i) => mask | (l ? 1 << i : 0), 0);
      return {
        q: Float64Array.from(current, (v, i) => clamp(v + dq[i] * dt, i)),
        flags: (sigma < WARN_SIGMA ? JOG_NEAR_SINGULARITY : 0) | (limited ? JOG_JOINT_LIMIT : 0) | (speedLimited ? JOG_SPEED_LIMITED : 0) | (drive > 0 ? JOG_ESCAPING : 0),
        limited,
      };
    },
  };
}
