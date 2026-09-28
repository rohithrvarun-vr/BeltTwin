// Node-RED function node: "scenario runner"  (Setup tab: Outputs = 2)
// Inputs:  "tick" every 0.5 s, "go" to start a campaign, "abort" to stop it.
// Output 1 -> "cmd to opcua" (plain-string commands, same whitelist as MQTT)
// Output 2 -> write file node, data\manifest.csv (append, no added newline)
// Setup tab -> Modules: add module "fs", variable "fs"  (needed for resume)
//
// RESUMABLE: on "go", runs already in manifest.csv with status ok and
// stale_ticks 0 are subtracted from the plan, so a crash or Node-RED restart
// only costs the run that was in progress.

// ---- dry run: set to null for the full campaign ----
const TEST_PLAN = null;

// ---- campaign definition ----
const SPEEDS = [40, 50, 60, 70, 80];
const FAULTS = ["jam", "slip", "overload", "wear"];
const REPS_PER_SPEED    = 4;   // x5 speeds = 20 runs per fault
const HEALTHY_PER_SPEED = 2;   // x2 kinds x5 speeds = 20 healthy runs

const FAULT_CODE = { jam: 1, slip: 2, wear: 3, overload: 4 };
const TIMEOUT_S  = { jam: 15, slip: 90, overload: 150, wear: 900 };  // ~2x measured trip time
const HOLD_MIN_S = 20, HOLD_MAX_S = 60;       // healthy interval before injection
const HEALTHY_S = 180, CHANGE_AT_S = 90;      // healthy runs
const SETTLE_TOL = 0.15, SETTLE_TICKS = 10;   // 10-s mean within 0.15 degC, held for 10 ticks (5 s)
const SETTLE_WIN = 20;                        // ticks in the rolling mean (20 x 0.5 s = 10 s)
const WARMUP_TIMEOUT_S = 1200;
const POST_TRIP_S = 10;
const STALE_MS = 3000;

// ---- state ----
let S = context.get("S") || { mode: "idle" };
const c = flow.get("cache") || {};
const now = Date.now();
const fresh = now - (flow.get("lastUpdate") || 0) < STALE_MS;
const cmds = [];
let manifest = null;

const send = s => cmds.push({ payload: s });
const setLabel = (phase, label) => { flow.set("phase", phase); flow.set("label", label); };
const secs = () => (now - S.t0) / 1000;
const go = step => { S.step = step; S.t0 = now; };

function shuffle(a) {
    for (let i = a.length - 1; i > 0; i--) {
        const j = Math.floor(Math.random() * (i + 1));
        [a[i], a[j]] = [a[j], a[i]];
    }
    return a;
}

const MANIFEST = "C:\\Users\\varun\\Documents\\TcXaeShell\\BeltTwin\\data\\manifest.csv";

function doneCounts() {
    const m = {};
    if (!fs.existsSync(MANIFEST)) return m;
    for (const line of fs.readFileSync(MANIFEST, "utf8").split(/\r?\n/)) {
        const c = line.split(",");
        if (c.length < 11 || c[0] === "run_id") continue;
        if (c[9] !== "ok" || Number(c[10] || 0) !== 0) continue;
        const key = c[1] + "@" + c[2];
        m[key] = (m[key] || 0) + 1;
    }
    return m;
}

function buildPlan() {
    if (TEST_PLAN) return TEST_PLAN.map(r => Object.assign({}, r));
    const done = doneCounts();
    const p = [];
    for (const sp of SPEEDS) {
        for (const f of FAULTS)
            for (let k = 0; k < REPS_PER_SPEED; k++) p.push({ kind: f, speed: sp });
        for (let k = 0; k < HEALTHY_PER_SPEED; k++) {
            p.push({ kind: "healthy", speed: sp });
            const others = SPEEDS.filter(x => x !== sp);
            p.push({ kind: "healthy_change", speed: sp,
                     speed2: others[Math.floor(Math.random() * others.length)] });
        }
    }
    const left = p.filter(r => {
        const key = r.kind + "@" + r.speed;
        if (done[key] > 0) { done[key]--; return false; }
        return true;
    });
    return shuffle(left);
}

function finish(status) {
    const r = S.run || {};
    const f = v => (v === undefined || v === null) ? "" : v;
    manifest = { payload: [f(r.id), f(r.kind), f(r.speed), f(r.speed2), f(r.hold),
                           f(r.injectTs), f(r.tripTs), f(r.tripCode), f(FAULT_CODE[r.kind]),
                           status, f(r.staleTicks), f(r.warmup_s)].join(",") + "\n" };
    S.idx++;
    S.run = null;
    go("prep");
}

// ---- control messages ----
if (msg.payload === "go") {
    if (S.mode === "running") { node.warn("runner already running"); return null; }
    S = { mode: "running", plan: buildPlan(), idx: 0, run: null };
    go("prep");
    context.set("S", S);
    node.status({ fill: "blue", shape: "dot", text: "starting, " + S.plan.length + " runs" });
    const header = fs.existsSync(MANIFEST) ? null
        : { payload: "run_id,kind,speed,speed2,hold_s,inject_ts,trip_ts,trip_code,expected_code,status,stale_ticks,warmup_s\n" };
    return [null, header];
}

if (msg.payload === "abort") {
    if (S.mode === "running") {
        send("stop");
        if (S.run) finish("aborted");
    }
    S.mode = "idle";
    flow.set("runId", "idle");
    setLabel("idle", "none");
    context.set("S", S);
    node.status({ fill: "grey", shape: "ring", text: "aborted" });
    return [cmds.length ? cmds : null, manifest];
}

// ---- tick ----
if (S.mode !== "running") return null;

if (!fresh) {
    if (S.run) S.run.staleTicks++;
    node.status({ fill: "red", shape: "ring", text: "stale data - paused" });
    context.set("S", S);
    return null;
}

if (S.idx >= S.plan.length) {
    S.mode = "idle";
    flow.set("runId", "idle");
    setLabel("idle", "none");
    context.set("S", S);
    node.status({ fill: "green", shape: "dot", text: "campaign done: " + S.plan.length + " runs" });
    return null;
}

const planned = S.plan[S.idx];
const st = c.eState;
const amb = (typeof c.rAmbientTemp === "number") ? c.rAmbientTemp : 22;

switch (S.step) {

case "prep":
    setLabel("reset", "none");
    if (st === 3) { if (secs() > 1) { send("reset"); S.t0 = now; } break; }
    if (st === 1 || st === 2) { if (secs() > 1) { send("stop"); S.t0 = now; } break; }
    if (st !== 0) break;
    S.run = Object.assign({}, planned, {
        id: "run_" + new Date(now).toISOString().replace(/[:.]/g, "-") + "_" + planned.kind + "_" + planned.speed,
        staleTicks: 0
    });
    flow.set("runId", S.run.id);
    flow.set("headerWritten:" + S.run.id, false);
    flow.set("lastPlcTime", null);
    setLabel("warmup", "none");
    send("speed " + S.run.speed);
    send("maintenance");
    S.tries = 0;
    go("arm");
    break;

case "arm":
    setLabel("warmup", "none");
    if (secs() < 1) break;
    if (Math.abs(c.rSpeedRequest - S.run.speed) > 0.01) {
        if (++S.tries > 3) { finish("speed_write_failed"); break; }
        send("speed " + S.run.speed);
        S.t0 = now;
        break;
    }
    send("start");
    S.lastStart = now;
    S.settle = 0;
    S.mBuf = [];
    S.bBuf = [];
    go("warmup");
    break;

case "warmup": {
    setLabel("warmup", "none");
    if (secs() > WARMUP_TIMEOUT_S) { send("stop"); finish("warmup_timeout"); break; }
    if (st === 3) { finish("unexpected_fault_in_warmup"); break; }
    if (st === 0 && now - S.lastStart > 3000) { send("start"); S.lastStart = now; break; }
    if (st !== 2) break;
    const mT = amb + 0.5 * S.run.speed;
    const bT = amb + 0.175 * S.run.speed;
    S.mBuf = S.mBuf.concat(c.rMotorTemp).slice(-SETTLE_WIN);
    S.bBuf = S.bBuf.concat(c.rBearingTemp).slice(-SETTLE_WIN);
    const avg = a => a.reduce((x, y) => x + y, 0) / a.length;
    const ok = S.mBuf.length === SETTLE_WIN
            && Math.abs(avg(S.mBuf) - mT) < SETTLE_TOL
            && Math.abs(avg(S.bBuf) - bT) < SETTLE_TOL;
    S.settle = ok ? S.settle + 1 : 0;
    if (S.settle >= SETTLE_TICKS) {
        S.run.warmup_s = Math.round(secs());
        S.run.hold = S.run.kind.startsWith("healthy")
            ? HEALTHY_S
            : Math.round(HOLD_MIN_S + Math.random() * (HOLD_MAX_S - HOLD_MIN_S));
        setLabel("healthy", "none");
        go("healthy");
    }
    break;
}

case "healthy":
    setLabel("healthy", "none");
    if (st !== 2) { finish("unexpected_state_" + st + "_in_healthy"); break; }
    if (S.run.kind === "healthy_change" && !S.run.changed && secs() >= CHANGE_AT_S) {
        send("speed " + S.run.speed2);
        S.run.changed = true;
    }
    if (secs() >= S.run.hold) {
        if (S.run.kind.startsWith("healthy")) {
            send("stop");
            finish("ok");
        } else {
            send(S.run.kind);
            S.run.injectTs = now;
            setLabel("developing", S.run.kind);
            go("developing");
        }
    }
    break;

case "developing":
    setLabel("developing", S.run.kind);
    if (st === 3) {
        S.run.tripTs = now;
        S.run.tripCode = c.eFaultCode;
        setLabel("faulted", S.run.kind);
        go("faulted");
        break;
    }
    if (st !== 2) { finish("unexpected_state_" + st + "_in_developing"); break; }
    if (secs() > TIMEOUT_S[S.run.kind]) { send("stop"); finish("no_trip_timeout"); }
    break;

case "faulted":
    setLabel("faulted", S.run.kind);
    if (secs() >= POST_TRIP_S) {
        send("reset");
        send("maintenance");
        finish(S.run.tripCode === FAULT_CODE[S.run.kind] ? "ok" : "wrong_code");
    }
    break;
}

node.status({ fill: "blue", shape: "dot",
    text: (S.idx + 1) + "/" + S.plan.length + " " + planned.kind + "@" + planned.speed +
          " " + S.step + " " + Math.round(secs()) + "s" });
context.set("S", S);
return [cmds.length ? cmds : null, manifest];
