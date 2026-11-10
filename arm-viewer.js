// The /arm/ page around the raylib viewer (c/viewer.c, built by
// `make -C c site` into viewer/). The C side draws the arm
// and its annotations and reads the mouse; this file owns the keyboard, the
// HTML controls, the tool readout, the status line and the screen-reader
// joint list, and passes input in through the module's web_* exports.
const MODULE_URL = new URL("./viewer/arm.mjs", import.meta.url).href;
const FILES = [
  ...["base_link", "1_Link", "2_Link", "3_Link", "4_Link", "5_Link", "6_Link"]
    .map((name) => [`meshes/${name}.glb`, new URL(`./meshes/${name}.glb`, import.meta.url).href]),
  ["DepartureMono.ttf", new URL("./viewer/DepartureMono.ttf", import.meta.url).href],
];
const JOINTS = 6;
const FRAME_ORDER = ["view", "base", "tool"];

const viewport = document.getElementById("arm-viewport");
const status = document.getElementById("arm-status");
const tcpReadout = Object.fromEntries(["x", "y", "z"].map((axis) => [axis, document.querySelector(`#arm-tcp [data-axis="${axis}"]`)]));
const frameButton = document.getElementById("arm-frame");
const frameName = frameButton.querySelector("[data-frame-name]");
const homeButton = document.getElementById("arm-home");
const jointsButton = document.getElementById("arm-joints-toggle");
const notesButton = document.getElementById("arm-notes-toggle");
const notes = document.getElementById("arm-notes");
const notesClose = document.getElementById("arm-notes-close");
const keycaps = [...document.querySelectorAll(".keycap:not(#arm-keypad-turn)")];
const turnSwitch = document.getElementById("arm-keypad-turn");
const jointList = document.getElementById("arm-joints");
const singularityText = document.getElementById("arm-singularity");
const held = new Set();

function setStatus(message, state) {
  status.textContent = message;
  if (state) status.dataset.state = state;
  else delete status.dataset.state;
}

function fail(error) {
  const message = error instanceof Error ? error.message : String(error);
  setStatus(`Arm unavailable: ${message}`, "error");
  const loading = viewport.querySelector(".viewer-loading");
  if (loading) loading.textContent = "render unavailable";
}

// First visit: the notes are open, so nobody has to guess the keys. After
// that they stay closed until asked for ("keys", or ?).
const NOTES_SEEN = "arm-notes-seen";
function setNotesOpen(open) {
  notes.hidden = !open;
  notesButton.setAttribute("aria-expanded", String(open));
  if (!open) {
    try { localStorage.setItem(NOTES_SEEN, "1"); } catch { /* storage may be blocked */ }
  }
}
let seenNotes = false;
try { seenNotes = localStorage.getItem(NOTES_SEEN) === "1"; } catch { /* storage may be blocked */ }
setNotesOpen(!seenNotes);
notesButton.addEventListener("click", () => setNotesOpen(notes.hidden));
notesClose.addEventListener("click", () => {
  setNotesOpen(false);
  viewport.focus({ preventScroll: true });
});

async function init() {
  const canvas = document.createElement("canvas");
  canvas.id = "arm-canvas";
  canvas.setAttribute("role", "img");
  canvas.setAttribute("aria-label", "One interactive six-axis GLUON robot arm");
  canvas.addEventListener("contextmenu", (event) => event.preventDefault());
  viewport.prepend(canvas);

  // raylib's web backend (Emscripten GLFW) listens on the whole window and
  // cancels Tab and Backspace. This listener is registered first, so it keeps
  // those two keys away from it and page navigation keeps working.
  addEventListener("keydown", (event) => {
    if (event.key === "Tab" || event.key === "Backspace") event.stopImmediatePropagation();
  }, true);

  const files = await Promise.all(FILES.map(async ([path, url]) => {
    const response = await fetch(url);
    if (!response.ok) throw new Error(`${url} returned ${response.status}`);
    return [path, new Uint8Array(await response.arrayBuffer())];
  }));
  const { default: createArmViewer } = await import(/* @vite-ignore */ MODULE_URL);
  const Module = {
    canvas,
    print: () => {},
    printErr: (text) => console.warn(text),
    preRun: [() => {
      Module.FS.mkdir("meshes");
      for (const [path, bytes] of files) Module.FS.writeFile(path, bytes);
    }],
  };
  const arm = await createArmViewer(Module);
  viewport.querySelector(".viewer-loading")?.remove();
  clearTimeout(window.__armLoadTimeout);

  const reduceMotion = matchMedia("(prefers-reduced-motion: reduce)");
  arm._web_set_reduce_motion(reduceMotion.matches ? 1 : 0);
  reduceMotion.addEventListener("change", () => arm._web_set_reduce_motion(reduceMotion.matches ? 1 : 0));

  const JOG_NEAR_SINGULARITY = 1;
  const JOG_JOINT_LIMIT = 2;
  const JOG_STATUS = [
    [JOG_JOINT_LIMIT, "joint limit", "joint limit reached · release, or jog another way"],
    [JOG_NEAR_SINGULARITY, "singular", "near a singularity · motion damped"],
  ];
  let controlFrame = "view";
  let jointsShown = false;
  let jogFlags = 0;
  let jogLimited = 0;
  let homing = false;

  const idleStatus = () => (homing ? "driving back to the home pose" : `${controlFrame} frame: hold W A S D, R F to move, I K J L U O to turn`);
  const reportJog = (flags, limited = 0) => {
    if (flags === jogFlags && limited === jogLimited) return;
    jogFlags = flags;
    jogLimited = limited;
    const hit = JOG_STATUS.find(([bit]) => flags & bit);
    setStatus(hit ? hit[2] : idleStatus(), hit ? "warn" : "ready");
  };

  // [right, forward, up] intent; the C side turns it into a tool velocity in
  // the chosen frame, eases it, and caps it at fine 0.04, normal 0.15 or fast
  // 0.35 m/s.
  const jogMap = {
    a: [0, -1], d: [0, 1], ArrowLeft: [0, -1], ArrowRight: [0, 1],
    w: [1, 1], s: [1, -1],
    r: [2, 1], f: [2, -1], ArrowUp: [2, 1], ArrowDown: [2, -1],
  };
  // Turns about [right, forward, up]: I K tip the tool down and up, J L turn
  // it left and right, U O roll it. While any is held the solver holds the
  // full pose, so the tool turns about its tip.
  const turnMap = {
    i: [0, -1], k: [0, 1],
    j: [2, 1], l: [2, -1],
    u: [1, -1], o: [1, 1],
  };
  const modifiers = { fine: false, fast: false };
  const keyOf = (event) => (event.key.length === 1 ? event.key.toLowerCase() : event.key);

  const goHome = () => {
    arm._web_home();
    homing = true;
    setStatus(idleStatus(), "ready");
  };
  // One button shows the frame the keys move along; it (and V) steps through
  // view, base and tool. B and T still jump straight to theirs.
  const setFrame = (name) => {
    controlFrame = name;
    arm._web_set_frame(FRAME_ORDER.indexOf(name));
    frameName.textContent = name;
    frameButton.setAttribute("aria-label", `Jog frame: ${name}. Press to change.`);
    setStatus(idleStatus(), "ready");
  };
  const nextFrame = () => setFrame(FRAME_ORDER[(FRAME_ORDER.indexOf(controlFrame) + 1) % FRAME_ORDER.length]);
  const setJointsShown = (on) => {
    jointsShown = on;
    arm._web_set_joints(on ? 1 : 0);
    jointsButton.setAttribute("aria-pressed", String(on));
  };

  // Keys work while the arm is focused. Hovering a mouse over it focuses it
  // (unless something else already has focus), so no click is needed.
  let hoverFocused = false;
  viewport.addEventListener("pointerdown", (event) => {
    if (event.target.closest("button, #arm-notes")) return;
    hoverFocused = false;
    viewport.focus({ preventScroll: true });
  });
  viewport.addEventListener("pointerenter", (event) => {
    if (event.pointerType !== "mouse") return;
    const active = document.activeElement;
    if (active && active !== document.body && active !== viewport) return;
    hoverFocused = active !== viewport;
    viewport.focus({ preventScroll: true });
  });
  viewport.addEventListener("pointerleave", () => {
    if (hoverFocused && document.activeElement === viewport) viewport.blur();
    hoverFocused = false;
  });
  viewport.addEventListener("blur", () => { held.clear(); });

  // On-screen controls. A pointer click hands focus back to the arm so the
  // keyboard keeps working; a keyboard activation keeps focus on the button.
  const refocus = (event) => { if (event.detail > 0) viewport.focus({ preventScroll: true }); };
  frameButton.addEventListener("click", (event) => { nextFrame(); refocus(event); });

  // The keycaps move the tool, or with "turn" on, turn it; each cap then sends
  // its I K J L U O twin and shows that letter.
  let keypadTurns = false;
  const moveLetters = new Map(keycaps.filter((cap) => cap.dataset.key).map((cap) => [cap, cap.textContent]));
  const setKeypadTurns = (on) => {
    keypadTurns = on;
    turnSwitch.setAttribute("aria-pressed", String(on));
    for (const [cap, letter] of moveLetters) {
      cap.textContent = on ? cap.dataset.turn.toUpperCase() : letter;
      cap.setAttribute("aria-label", on ? cap.dataset.turnLabel : cap.dataset.moveLabel);
    }
  };
  turnSwitch.addEventListener("click", (event) => { setKeypadTurns(!keypadTurns); refocus(event); });
  homeButton.addEventListener("click", (event) => { goHome(); refocus(event); });
  jointsButton.addEventListener("click", (event) => { setJointsShown(!jointsShown); refocus(event); });

  // Keycaps hold a jog key (or a speed modifier) for as long as the pointer is
  // down, which also makes the arm drivable by touch.
  const releasePointerKeys = new Map();
  for (const keycap of keycaps) {
    // A held keycap is a button, not text: no selection, magnifier, callout
    // or context menu on a long press.
    keycap.addEventListener("touchstart", (event) => event.preventDefault(), { passive: false });
    keycap.addEventListener("contextmenu", (event) => event.preventDefault());
    keycap.addEventListener("pointerdown", (event) => {
      event.preventDefault();
      keycap.setPointerCapture(event.pointerId);
      viewport.focus({ preventScroll: true });
      if (keycap.dataset.key) {
        const keyName = keypadTurns ? keycap.dataset.turn : keycap.dataset.key;
        held.add(keyName);
        releasePointerKeys.set(event.pointerId, () => held.delete(keyName));
      } else {
        modifiers[keycap.dataset.modifier] = true;
        releasePointerKeys.set(event.pointerId, () => { modifiers[keycap.dataset.modifier] = false; });
      }
    });
    const release = (event) => {
      releasePointerKeys.get(event.pointerId)?.();
      releasePointerKeys.delete(event.pointerId);
    };
    keycap.addEventListener("pointerup", release);
    keycap.addEventListener("pointercancel", release);
    keycap.addEventListener("lostpointercapture", release);
  }

  addEventListener("keydown", (event) => {
    if (event.metaKey || event.ctrlKey || event.altKey) return;
    // ? opens the notes from anywhere on the page; Escape closes them.
    if (event.key === "?") { setNotesOpen(notes.hidden); return; }
    if (event.key === "Escape" && !notes.hidden) { setNotesOpen(false); return; }
    if (document.activeElement !== viewport) return;
    modifiers.fine = event.shiftKey;
    const keyName = keyOf(event);
    if (keyName === " ") {
      event.preventDefault();
      modifiers.fast = true;
    } else if (keyName === "v") {
      event.preventDefault();
      nextFrame();
    } else if (keyName === "b" || keyName === "t") {
      event.preventDefault();
      setFrame(keyName === "b" ? "base" : "tool");
    } else if (keyName === "x") {
      event.preventDefault();
      setJointsShown(!jointsShown);
    } else if (keyName === "h") {
      event.preventDefault();
      goHome();
    } else if (keyName in jogMap || keyName in turnMap) {
      event.preventDefault();
      held.add(keyName);
    }
  });
  addEventListener("keyup", (event) => {
    modifiers.fine = event.shiftKey;
    const keyName = keyOf(event);
    if (keyName === " ") modifiers.fast = false;
    held.delete(keyName);
  });
  addEventListener("blur", () => {
    held.clear();
    modifiers.fine = modifiers.fast = false;
  });

  const rows = Array.from({ length: JOINTS }, () => jointList.appendChild(document.createElement("li")));
  const rowCache = [];
  const formatMetres = (value) => `${value < 0 ? "−" : ""}${Math.abs(value).toFixed(3)}`;
  // Arrow keys light the same keycaps as R/F and A/D; turn keys light their
  // twin caps while the keypad shows turning.
  const KEYCAP_ALIAS = { ArrowLeft: "a", ArrowRight: "d", ArrowUp: "r", ArrowDown: "f" };
  const CAP_FOR_TURN = Object.fromEntries(keycaps.filter((cap) => cap.dataset.turn).map((cap) => [cap.dataset.turn, cap.dataset.key]));
  let keycapState = "";
  const syncKeycaps = () => {
    const active = new Set([...held]
      .filter((name) => (name in turnMap) === keypadTurns)
      .map((name) => CAP_FOR_TURN[name] ?? KEYCAP_ALIAS[name] ?? name));
    const state = `${[...active].sort().join("")}|${modifiers.fine}|${modifiers.fast}|${keypadTurns}`;
    if (state === keycapState) return;
    keycapState = state;
    for (const keycap of keycaps) {
      const on = keycap.dataset.key ? active.has(keycap.dataset.key) : modifiers[keycap.dataset.modifier];
      keycap.classList.toggle("on", Boolean(on));
    }
  };

  const tick = () => {
    const intent = [0, 0, 0], turn = [0, 0, 0];
    for (const keyName of held) {
      const [axis, direction] = jogMap[keyName] ?? turnMap[keyName];
      (keyName in turnMap ? turn : intent)[axis] += direction;
    }
    arm._web_input(intent[0], intent[1], intent[2], turn[0], turn[1], turn[2], modifiers.fine ? 1 : 0, modifiers.fast ? 1 : 0);

    // web_state() refreshes and returns the C side's state block.
    const statePointer = arm._web_state() / 8;
    const state = arm.HEAPF64.subarray(statePointer, statePointer + 3 + JOINTS + 5);
    const [x, y, z] = state;
    const flags = state[3 + JOINTS], limited = state[4 + JOINTS], trust = state[5 + JOINTS], near = state[6 + JOINTS];
    tcpReadout.x.textContent = formatMetres(x);
    tcpReadout.y.textContent = formatMetres(y);
    tcpReadout.z.textContent = formatMetres(z);
    const nowHoming = state[7 + JOINTS] === 1;
    if (homing && !nowHoming && !held.size) setStatus("at the home pose", "ready");
    homing = nowHoming;
    reportJog(flags, limited);
    for (let index = 0; index < JOINTS; index += 1) {
      const at = (limited >> index) & 1, close = (near >> index) & 1;
      const text = `Joint ${index + 1}: ${String(Math.round(state[3 + index] * 180 / Math.PI))}°${at ? ", at its limit" : close ? ", near its limit" : ""}`;
      if (text !== rowCache[index]) rows[index].textContent = rowCache[index] = text;
    }
    const proximity = 1 - Math.min(1, Math.max(0, trust));
    const word = proximity === 0 ? "clear" : proximity < 0.75 ? "near" : "at";
    if (singularityText.textContent !== word) singularityText.textContent = word;
    syncKeycaps();
    requestAnimationFrame(tick);
  };
  requestAnimationFrame(tick);
  setStatus(idleStatus(), "ready");
}

init().catch(fail);
