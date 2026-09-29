// Node-RED function node: "scenario runner"  v3  (Setup tab: Outputs = 2, Modules: fs -> fs)
// Inputs:  "tick" every 0.5 s, "go" = full campaign, "go dry" = 4-run dry run, "abort".
// Output 1 -> "cmd to opcua" (plain-string commands)
// Output 2 -> manifest write file node (filename taken from msg.filename)
//
// v3 changes:
//  - data goes to data\v3\ (campaign) or data\v3dry\ (dry run); v2 manifest is never read
//  - every inject verified via nInjectCount (+1 within 2 s, max 3 tries, double inject detected)
//  - every maintenance verified via nMaintenanceCount (max 3 tries)
//  - settle = slope-based (60 s regression slope), not distance to a computed target
//  - timeouts cover randomised onset (x0.5..x2 nominal)
//  - 150-run plan: 25 per fault, 20 healthy, 20 healthy_change, 10 x 30-min soak
// RESUMABLE: on "go", runs in data\v3\manifest.csv with status ok and stale_ticks 0 are
// subtracted from the plan.

const BASE = "C:\\Users\\varun\\Documents\\TcXaeShell\\BeltTwin\\data\\";

const TEST_PLAN = [
    { kind: "jam", speed: 80 },
    { kind: "slip", speed: 60 },
    { kind: "overload", speed: 40 },
    { kind: "healthy_change", speed: 60, speed2: 40 }
];

// ---- campaign definition ----
const SPEEDS = [40, 50, 60, 70, 80];
const FAULTS = ["jam", "slip", "overload", "wear"];
const REPS_PER_SPEED    = 5;   // x5 speeds = 25 runs per fault
const HEALTHY_PER_SPEED = 4;   // x5 speeds = 20 healthy + 20 healthy_change
const SOAK_PER_SPEED    = 2;   // x5 speeds = 10 soak runs

const FAULT_CODE = { jam: 1, slip: 2, wear: 3, overload: 4 };
const TIMEOUT_S  = { jam: 15, slip: 90, overload: 150, wear: 1000 };  // max onset x2 + margin
const HOLD_MIN_S = 20, HOLD_MAX_S = 60;       // healthy interval before injection
const HEALTHY_S = 180, CHANGE_AT_S = 90, SOAK_S = 1800;
const TICK_S = 0.5;
const SLOPE_WIN = 120;                        // 120 ticks = 60 s regression window
const MIN_WARMUP_S = 240;
const SLOPE_MOTOR = 0.01, SLOPE_BEAR = 0.004; // degC/s, both held for SETTLE_TICKS
const SETTLE_TICKS = 10;
const WARMUP_TIMEOUT_S = 1200;
const POST_TRIP_S = 10;
const STALE_MS = 3000;
const VERIFY_MS = 2000, MAX_TRIES = 3;

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
const dirOf = dry => BASE + (dry ? "v3dry\\" : "v3\\");
const manifestOf = dry => dirOf(dry) + "manifest.csv";
const HEADER = "run_id,kind,speed,speed2,hold_s,inject_ts,trip_ts,trip_code,expected_code,status,stale_ticks,warmup_s,inject_tries,maint_tries\n";

function shuffle(a) {
    for (let i = a.length - 1; i > 0; i--) {
        const j = Math.floor(Math.random() * (i + 1));
        [a[i], a[j]] = [a[j], a[i]];
    }
    return a;
}

function slope(a) {
    const n = a.length;
    const xm = (n - 1) / 2;
    const ym = a.reduce((x, y) => x + y, 0) / n;
    let sxy = 0, sxx = 0;
    for (let i = 0; i < n; i++) {
        const dx = i - xm;
        sxy += dx * (a[i] - ym);
        sxx += dx * dx;
    }
    return sxy / sxx / TICK_S;
}

function doneCounts(file) {
    const m = {};
    if (!fs.existsSync(file)) return m;
    for (const line of fs.readFileSync(file, "utf8").split(/\r?\n/)) {
        const col = line.split(",");
        if (col.length < 11 || col[0] === "run_id") continue;
        if (col[9] !== "ok" || Number(col[10] || 0) !== 0) continue;
        const key = col[1] + "@" + col[2];
        m[key] = (m[key] || 0) + 1;
    }
    return m;
}

function buildPlan(dry) {
    if (dry) return TEST_PLAN.map(r => Object.assign({}, r));
    const done = doneCounts(manifestOf(false));
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
        for (let k = 0; k < SOAK_PER_SPEED; k++) p.push({ kind: "soak", speed: sp });
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
    manifest = {
        filename: manifestOf(S.dry),
        payload: [f(r.id), f(r.kind), f(r.speed), f(r.speed2), f(r.hold),
                  f(r.injectTs), f(r.tripTs), f(r.tripCode), f(FAULT_CODE[r.kind]),
                  status, f(r.staleTicks), f(r.warmup_s), f(r.injTries), f(r.maintTries)].join(",") + "\n"
    };
    S.idx++;
    S.run = null;
    go("prep");
}

// ---- control messages ----
if (msg.payload === "go" || msg.payload === "go dry") {
    if (S.mode === "running") { node.warn("runner already running"); return null; }
    if (typeof c.nInjectCount !== "number" || typeof c.nMaintenanceCount !== "number") {
        node.warn("nInjectCount / nMaintenanceCount not in cache: fan-out not updated or no data");
        node.status({ fill: "red", shape: "ring", text: "refused: no nInjectCount" });
        return null;
    }
    const dry = msg.payload === "go dry";
    S = { mode: "running", dry: dry, plan: buildPlan(dry), idx: 0, run: null };
    flow.set("dataDir", dirOf(dry));
    go("prep");
    context.set("S", S);
    node.status({ fill: "blue", shape: "dot", text: (dry ? "DRY " : "") + "starting, " + S.plan.length + " runs" });
    const mf = manifestOf(dry);
    const header = fs.existsSync(mf) ? null : { filename: mf, payload: HEADER };
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
    node.status({ fill: "green", shape: "dot", text: (S.dry ? "DRY " : "") + "campaign done: " + S.plan.length + " runs" });
    return null;
}

const planned = S.plan[S.idx];
const st = c.eState;

switch (S.step) {

case "prep":
    setLabel("reset", "none");
    if (st === 3) { if (secs() > 1) { send("reset"); S.t0 = now; } break; }
    if (st === 1 || st === 2) { if (secs() > 1) { send("stop"); S.t0 = now; } break; }
    if (st !== 0) break;
    if (secs() < 1) break;                    // let counters from the previous run settle in the cache
    S.run = Object.assign({}, planned, {
        id: "run_" + new Date(now).toISOString().replace(/[:.]/g, "-") + "_" + planned.kind + "_" + planned.speed,
        staleTicks: 0,
        maintBase: c.nMaintenanceCount,
        maintTries: 1,
        injTries: 0
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
        if (++S.tries > MAX_TRIES) { finish("speed_write_failed"); break; }
        send("speed " + S.run.speed);
        S.t0 = now;
        break;
    }
    if (!(c.nMaintenanceCount > S.run.maintBase)) {
        if (secs() * 1000 < VERIFY_MS) break;
        if (S.run.maintTries >= MAX_TRIES) { finish("maint_write_failed"); break; }
        send("maintenance");
        S.run.maintTries++;
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
    S.mBuf = S.mBuf.concat(c.rMotorTemp).slice(-SLOPE_WIN);
    S.bBuf = S.bBuf.concat(c.rBearingTemp).slice(-SLOPE_WIN);
    const ok = secs() >= MIN_WARMUP_S
            && S.mBuf.length === SLOPE_WIN
            && Math.abs(slope(S.mBuf)) < SLOPE_MOTOR
            && Math.abs(slope(S.bBuf)) < SLOPE_BEAR;
    S.settle = ok ? S.settle + 1 : 0;
    if (S.settle >= SETTLE_TICKS) {
        S.run.warmup_s = Math.round(secs());
        if (S.run.kind === "soak") S.run.hold = SOAK_S;
        else if (S.run.kind.startsWith("healthy")) S.run.hold = HEALTHY_S;
        else S.run.hold = Math.round(HOLD_MIN_S + Math.random() * (HOLD_MAX_S - HOLD_MIN_S));
        S.mBuf = [];
        S.bBuf = [];
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
        if (S.run.kind === "healthy" || S.run.kind === "healthy_change" || S.run.kind === "soak") {
            send("stop");
            finish("ok");
        } else {
            S.run.injBase = c.nInjectCount;
            S.run.injTries = 1;
            S.run.injOk = false;
            S.injSent = now;
            send(S.run.kind);
            S.run.injectTs = now;
            setLabel("developing", S.run.kind);
            go("developing");
        }
    }
    break;

case "developing": {
    setLabel("developing", S.run.kind);
    const dn = c.nInjectCount - S.run.injBase;
    if (dn > 1) { send("stop"); finish("double_inject"); break; }
    if (dn === 1) S.run.injOk = true;
    if (st === 3) {
        S.run.tripTs = now;
        S.run.tripCode = c.eFaultCode;
        setLabel("faulted", S.run.kind);
        go("faulted");
        break;
    }
    if (st !== 2) { finish("unexpected_state_" + st + "_in_developing"); break; }
    if (!S.run.injOk && now - S.injSent > VERIFY_MS) {
        if (S.run.injTries >= MAX_TRIES) { send("stop"); finish("inject_write_failed"); break; }
        send(S.run.kind);
        S.run.injTries++;
        S.injSent = now;
        S.run.injectTs = now;
        S.t0 = now;                            // timeout counts from the latest inject
        break;
    }
    if (secs() > TIMEOUT_S[S.run.kind]) { send("stop"); finish("no_trip_timeout"); }
    break;
}

case "faulted":
    setLabel("faulted", S.run.kind);
    if (secs() >= POST_TRIP_S) {
        const dn = c.nInjectCount - S.run.injBase;
        send("reset");
        send("maintenance");
        if (dn > 1) finish("double_inject");
        else if (dn < 1) finish("inject_unverified");
        else finish(S.run.tripCode === FAULT_CODE[S.run.kind] ? "ok" : "wrong_code");
    }
    break;
}

node.status({ fill: "blue", shape: "dot",
    text: (S.dry ? "DRY " : "") + (S.idx + 1) + "/" + S.plan.length + " " + planned.kind + "@" + planned.speed +
          " " + S.step + " " + Math.round(secs()) + "s" });
context.set("S", S);
return [cmds.length ? cmds : null, manifest];
