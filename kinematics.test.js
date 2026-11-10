// node --test: the JS kinematics against fixtures recorded from the retired Rust/WASM build.
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { test } from "node:test";
import { createKinematics, jacobian, JOG_ESCAPING } from "./kinematics.js";

const read = (name) => readFileSync(new URL(name, import.meta.url), "utf8");
const fixture = JSON.parse(read("./kinematics.fixture.json"));
const joints = [...read("./gluon.urdf").matchAll(/<joint[\s\S]*?<\/joint>/g)].map((m) => m[0])
  .filter((xml) => /type="revolute"/.test(xml)).map((xml) => {
    const triple = (re) => xml.match(re)[1].trim().split(/\s+/).map(Number);
    const value = (name) => Number(xml.match(new RegExp(`${name}="([^"]+)"`))[1]);
    return { origin: triple(/<origin\s+xyz="([^"]+)"/), axis: triple(/<axis\s+xyz="([^"]+)"/), lower: value("lower"), upper: value("upper"), velocity: value("velocity") };
  });
const HOME = [0, -0.45, 0.95, 0, 0.55, 0];
const close = (actual, expected, tol, label) => {
  for (let i = 0; i < expected.length; i += 1) assert.ok(Math.abs(actual[i] - expected[i]) <= tol, `${label}[${i}]: ${actual[i]} vs ${expected[i]}`);
};
const engine = (positionOnly) => { const k = createKinematics(joints, HOME); k.setPositionOnly(positionOnly); return k; };

test("FK and trust match the Rust fixture", () => {
  assert.equal(joints.length, 6);
  for (const row of fixture.fk) for (const [mode, positionOnly] of [[0, true], [1, false]]) {
    const out = engine(positionOnly).solve(row.q);
    close(out.links, row.links, 1e-12, `links ${row.q}`);
    close(out.tcp, row.tcp, 1e-12, `tcp ${row.q}`);
    close([out.trust], [row.trust[mode]], 1e-9, `trust ${row.q} positionOnly=${positionOnly}`);
  }
});

test("FK matches the independent Python oracle", () => {
  const poses = [
    [HOME, [0.075661948, -0.326696982, 0.305343404], [[0.852524522060, 0.522687228931, 0], [-0.088839654932, 0.144901157267, -0.985449729988], [-0.515081988618, 0.840120060072, 0.169967142900]]],
    [[0, 0, 0, 0, 0, 0], [0.075685673, -0.000022696, 0.533979000], [[1, 0, 0], [0, 1, 0], [0, 0, 1]]],
    [[0.3, -0.5, 0.8, 0.2, -0.4, 0.6], [-0.023166359, -0.331041311, 0.341080001], [[0.932123466540, -0.056435865160, -0.357716139207], [-0.103442787863, 0.905134673347, -0.412347926809], [0.347052492808, 0.421362333661, 0.837859386178]]],
    [[-1, 0.7, -1.2, 0.5, 1, -0.8], [-0.238108309, 0.242822664, 0.122997961], [[-0.230202307050, 0.142598520700, -0.962638332772], [0.789903817462, 0.605145252162, -0.099253125611], [0.568382667740, -0.783239992390, -0.251944949014]]],
  ];
  for (const [q, position, rotation] of poses) {
    const { links, tcp } = engine(true).solve(q);
    close(tcp, position, 1e-6, `tcp ${q}`);
    const [x, y, z, w] = links.slice(38, 42);
    const R = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]];
    R.forEach((row, r) => close(row, rotation[r], 1e-9, `rotation row ${r} ${q}`));
  }
});

test("Jacobian matches central differences", () => {
  const q = [0.2, -0.3, 0.6, 0.4, -0.5, 0.1], eps = 1e-6, J = jacobian(joints, q), k = engine(true);
  for (let i = 0; i < 6; i += 1) {
    const plus = k.solve(q.map((v, j) => v + (i === j ? eps : 0))).tcp, minus = k.solve(q.map((v, j) => v - (i === j ? eps : 0))).tcp;
    close([0, 1, 2].map((r) => J[r][i]), [0, 1, 2].map((r) => (plus[r] - minus[r]) / (2 * eps)), 1e-5, `column ${i}`);
  }
});

// Replays a recorded run and checks each checkpoint against the Rust fixture.
// The Rust build stayed stuck at the fully stretched pose when commanded back
// toward the shoulder; the escape added here is the one intended difference, so
// comparison stops at the first step that reports it.
const run = (positionOnly, start, steps, twistAt, dtAt, checkpoints) => {
  const k = engine(positionOnly);
  let q = start.slice(), flagsOr = 0, c = 0;
  for (let n = 1; n <= steps; n += 1) {
    const step = k.jog(q, twistAt(n - 1), dtAt(n - 1));
    if (step.flags & JOG_ESCAPING) return { escaped: true };
    q = step.q; flagsOr |= step.flags;
    const want = checkpoints[c];
    if (want && want.step === n) {
      c += 1;
      close(step.q, want.q, 1e-9, `q at step ${n}`);
      assert.deepEqual([step.flags, step.limited], [want.flags, want.limited], `flags/limited at step ${n}`);
    }
  }
  assert.equal(c, checkpoints.length);
  return { flagsOr };
};

test("jog checkpoints match the Rust fixture until the stuck-arm escape", () => {
  let escaped = 0;
  const check = (result, r) => { if (result.escaped) escaped += 1; else assert.equal(result.flagsOr, r.flagsOr); };
  for (const r of fixture.jog) check(run(r.positionOnly, r.start, 600, () => [...r.twist, 0, 0, 0], () => fixture.jogDt, r.checkpoints), r);
  const dts = [1 / 60, 1 / 144, 1 / 30, 0.05];
  for (const r of fixture.varying) {
    const twist = (f) => [0.35 * Math.cos(f / 90), 0.35 * Math.sin(f / 70), 0.2 * Math.sin(f / 50), 0, 0, 0];
    check(run(r.positionOnly, r.start, 2400, twist, (f) => dts[f % 4], r.checkpoints), r);
  }
  // Four runs start at the straight pose and are commanded back toward the
  // shoulder, and one long run wanders into it. Every other run is pinned in full.
  assert.equal(escaped, 5);
});

test("an arm pushed to full stretch can come back down", () => {
  for (const positionOnly of [true, false]) {
    const k = engine(positionOnly);
    let q = HOME.slice();
    for (let frame = 0; frame < 720; frame += 1) q = k.jog(q, [0, 0, 0.35, 0, 0, 0], 1 / 60).q;
    assert.ok(k.solve(q).trust < 0.01, "the arm should be at the singularity");
    const top = k.solve(q).tcp[2];
    for (let frame = 0; frame < 360; frame += 1) {
      q = k.jog(q, [0, 0, -0.35, 0, 0, 0], 1 / 60).q;
      q.forEach((v, i) => assert.ok(v >= joints[i].lower && v <= joints[i].upper, `joint ${i} left its limits`));
    }
    assert.ok(top - k.solve(q).tcp[2] > 0.3, `positionOnly=${positionOnly}: the tool came down only ${(top - k.solve(q).tcp[2]).toFixed(3)} m`);
  }
});

test("held keys stay inside limits and under the speed cap", () => {
  for (const positionOnly of [true, false]) for (let axis = 0; axis < 6; axis += 1) {
    const k = engine(positionOnly), twist = [0, 0, 0, 0, 0, 0];
    twist[axis >> 1] = axis & 1 ? -0.35 : 0.35;
    let q = HOME.slice(), flagsOr = 0;
    for (let frame = 0; frame < 3600; frame += 1) {
      const step = k.jog(q, twist, 1 / 60);
      step.q.forEach((v, i) => {
        assert.ok(v >= joints[i].lower && v <= joints[i].upper, `joint ${i} left its limits`);
        assert.ok(Math.abs(v - q[i]) <= (3.14 * 0.6) / 60 + 1e-9, `joint ${i} moved too fast`);
      });
      q = step.q; flagsOr |= step.flags;
    }
    assert.notEqual(flagsOr, 0, `axis ${axis} never reported a limit`);
  }
});

test("an arm pinned at a joint limit can back out", () => {
  const k = engine(true);
  let q = HOME.slice(), step;
  for (let frame = 0; frame < 900; frame += 1) q = (step = k.jog(q, [-0.35, 0, 0, 0, 0, 0], 1 / 60)).q;
  assert.notEqual(step.limited, 0);
  const pinned = Array.from(q);
  for (let frame = 0; frame < 60; frame += 1) q = k.jog(q, [0.35, 0, 0, 0, 0, 0], 1 / 60).q;
  assert.ok(Math.max(...pinned.map((v, i) => Math.abs(v - q[i]))) > 0.05);
});

test("non-finite input and dt = 0 are rejected", () => {
  const k = engine(true), q = HOME.slice();
  assert.equal(k.jog(q, Array(6).fill(NaN), 1 / 60), null);
  assert.equal(k.jog(q, [0.1, 0, 0, 0, 0, 0], 0), null);
  assert.equal(k.jog([NaN, ...q.slice(1)], [0.1, 0, 0, 0, 0, 0], 1 / 60), null);
  assert.deepEqual(q, HOME);
});
