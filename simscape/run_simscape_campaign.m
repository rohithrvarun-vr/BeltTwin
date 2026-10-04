function run_simscape_campaign(mode)
% run_simscape_campaign.m  (BeltTwin option A, step 5b)
% Usage:  run_simscape_campaign('dry')    5 runs  -> data\simscape_dry\   (check first)
%         run_simscape_campaign('full')   150 runs -> data\simscape\      (resumable)
%
% Runs the parameterised Simscape model 'conveyor_sim' (built by build_conveyor_step5) and
% writes each run as a CSV in the same format as the PLC dataset data\v3, plus a manifest.
% Same plan as PLC campaign v3: 25 per fault type (jam, slip, overload, wear), 20 healthy,
% 20 healthy with a speed change, 10 soak runs of 30 min; speeds 40..80; shuffled.
%
% Per run: 420 s warm-up, a healthy hold (faults 20-60 s, healthy 180 s, soak 1800 s), then
% the fault with randomised onset (time to trip = nominal x log-uniform [0.5, 2]), logged
% until 10 s after the trip.
% Sensors as PLC sensor model v3: current noise 0.05 A, 0.01 A resolution, rare spikes;
% temperatures noise 0.08 degC, 0.1 degC resolution; ambient 0.05 degC, 0.1 degC;
% belt position quantised to 0.25 units. VIBRATION IS NOT MODELLED: rVibration is NaN.
% Sampling 4 Hz. Resumable: runs already in the manifest are skipped.

if nargin < 1, mode = 'dry'; end
here = fileparts(mfilename('fullpath'));
cd(here);
mdl = 'conveyor_sim';
if ~exist([mdl '.slx'], 'file'), build_conveyor_step5(); end
load_system(mdl);

dataDir = fullfile(here, '..', 'data', ternary(strcmp(mode, 'full'), 'simscape', 'simscape_dry'));
if ~exist(dataDir, 'dir'), mkdir(dataDir); end
manifest = fullfile(dataDir, 'manifest.csv');
logFile = fullfile(dataDir, 'campaign_log.txt');
diary(logFile); cleanup = onCleanup(@() diary('off'));
fprintf('run_simscape_campaign(%s)  %s\n', mode, datestr(now));

%% ---------------- plan ----------------
Tnom = struct('jam', 2, 'slip', 25, 'overload', 50, 'wear', 400);
code = struct('jam', 1, 'slip', 2, 'wear', 3, 'overload', 4);
speeds = [40 50 60 70 80];
rng(20261004);
plan = struct('kind', {}, 'speed', {}, 'speed2', {});
if strcmp(mode, 'full')
    for sp = speeds
        for f = {'jam', 'slip', 'overload', 'wear'}
            for k = 1:5, plan(end+1) = struct('kind', f{1}, 'speed', sp, 'speed2', NaN); end %#ok<AGROW>
        end
        for k = 1:4
            plan(end+1) = struct('kind', 'healthy', 'speed', sp, 'speed2', NaN); %#ok<AGROW>
            others = speeds(speeds ~= sp);
            plan(end+1) = struct('kind', 'healthy_change', 'speed', sp, 'speed2', others(randi(4))); %#ok<AGROW>
        end
        for k = 1:2, plan(end+1) = struct('kind', 'soak', 'speed', sp, 'speed2', NaN); end %#ok<AGROW>
    end
    plan = plan(randperm(numel(plan)));
else
    plan = struct('kind', {'jam', 'slip', 'overload', 'healthy_change', 'wear'}, ...
                  'speed', {80, 60, 40, 60, 50}, 'speed2', {NaN, NaN, NaN, 40, NaN});
end
% per-run random draws, fixed by the seed above so a resumed campaign repeats them exactly
R = struct('hold', {}, 'factor', {}, 'seeds', {}, 'base', {});
for r = 1:numel(plan)
    R(r).hold = 20 + 40 * rand;
    R(r).factor = 2 ^ (2 * rand - 1);
    R(r).seeds = randi([1 2^31 - 2], 1, 4);
    R(r).base = randi([0 2^31]);
end

done = {};
if exist(manifest, 'file')
    try
        m = readtable(manifest, 'Delimiter', ',', 'TextType', 'string');
        done = cellstr(m.run_id(m.status == "ok"));
    catch
        done = {};                                   % manifest with header only
    end
else
    fid = fopen(manifest, 'w');
    fprintf(fid, 'run_id,kind,speed,speed2,hold_s,inject_ts,trip_ts,trip_code,expected_code,status,stale_ticks,warmup_s,inject_tries,maint_tries\n');
    fclose(fid);
end
fprintf('%d runs planned, %d already done\n\n', numel(plan), numel(done));

%% ---------------- runs ----------------
warm = 420;
t0all = tic; nRun = 0;
for r = 1:numel(plan)
    p = plan(r); q = R(r);
    rid = sprintf('sim_%03d_%s_%d', r, p.kind, p.speed);
    if any(strcmp(done, rid)), continue; end
    isFault = isfield(Tnom, p.kind);
    if isFault
        hold_s = q.hold;  T = Tnom.(p.kind) * q.factor;  tinj = warm + hold_s;  tstop = tinj + T + 10;
    else
        hold_s = ternary(strcmp(p.kind, 'soak'), 1800, 180);
        T = 1;  tinj = 1e9;  tstop = warm + hold_s;
    end
    du = 0; tchg = 1e9;
    if strcmp(p.kind, 'healthy_change'), du = p.speed2 - p.speed; tchg = warm + 90; end

    in = Simulink.SimulationInput(mdl);
    vars = {'p_u1', p.speed; 'p_du', du; 'p_tchg', tchg; 'p_tinj', tinj; 'p_T', T; ...
            'p_exp', ternary(strcmp(p.kind, 'jam'), 1, 2); ...
            'p_isjam', double(strcmp(p.kind, 'jam')); 'p_isslip', double(strcmp(p.kind, 'slip')); ...
            'p_isov', double(strcmp(p.kind, 'overload')); 'p_iswear', double(strcmp(p.kind, 'wear')); ...
            'p_seed_f', q.seeds(1); 'p_seed_s', q.seeds(2); 'p_seed_a', q.seeds(3); 'p_tstop', tstop};
    for v = 1:size(vars, 1), in = in.setVariable(vars{v, 1}, vars{v, 2}); end
    tic;
    out = sim(in);
    simSec = toc;

    % ---- resample to 4 Hz and build the PLC-format table
    t = (0:0.25:tstop)';
    g = @(name) rs(out.get(name), t);
    spd = g('speed_u'); cur = g('current_A'); tm = g('Tmot_C'); tb = g('Tbrg_C'); ta = g('Tamb_C');
    pos = g('belt_pos_m'); run_ = g('running');
    rn = rng; rng(q.seeds(4));                                  % sensor noise, reproducible per run
    n = numel(t);
    req = p.speed + du * (t >= tchg);
    kTrip = find(run_ < 0.5, 1);
    tripT = ternary(isempty(kTrip), NaN, t(max(kTrip, 1)));
    started = t >= 1;
    kReach = find(started & spd >= p.speed - 0.5, 1);
    state = 2 * ones(n, 1);
    state(~started) = 0;
    if ~isempty(kReach), state(started & (1:n)' < kReach) = 1; end
    if ~isnan(tripT), state(t >= tripT) = 3; end
    on = state == 1 | state == 2;
    spikes = (rand(n, 1) < 1e-4) .* (0.5 + rand(n, 1)) .* sign(rand(n, 1) - 0.5) .* on;
    curN = round((cur + 0.05 * randn(n, 1) .* on + spikes) / 0.01) * 0.01;
    tmN  = round((tm + 0.08 * randn(n, 1)) / 0.1) * 0.1;
    tbN  = round((tb + 0.08 * randn(n, 1)) / 0.1) * 0.1;
    taN  = round((ta + 0.05 * randn(n, 1)) / 0.1) * 0.1;
    posN = mod(round(abs(pos) * 100 / 0.25) * 0.25, 1000);
    rng(rn);
    inj = double(t >= tinj);
    faultCode = zeros(n, 1);
    if isFault && ~isnan(tripT), faultCode(state == 3) = code.(p.kind); end
    phase = repmat("healthy", n, 1);
    phase(t < warm) = "warmup";
    phase(t >= tinj) = "developing";
    phase(state == 3) = "faulted";
    label = repmat("none", n, 1);
    if isFault, label(t >= tinj) = string(p.kind); end
    ts0 = 1790000000000 + r * 1e7;
    ts = ts0 + round(t * 1000);
    plc = mod(q.base + round(t * 100) * 10, 2^32);

    Tbl = table(repmat(string(rid), n, 1), ts, plc, state, faultCode, spd, req, posN, curN, tmN, tbN, ...
        nan(n, 1), taN, double(started), zeros(n, 1), zeros(n, 1), double(state == 3), ones(n, 1), inj, phase, label, ...
        'VariableNames', {'run_id', 'ts', 'nPlcTime', 'eState', 'eFaultCode', 'rSpeed', 'rSpeedRequest', 'rPosition', ...
        'rMotorCurrent', 'rMotorTemp', 'rBearingTemp', 'rVibration', 'rAmbientTemp', 'nStartCount', 'nStopCount', ...
        'nResetCount', 'nFaultCount', 'nMaintenanceCount', 'nInjectCount', 'phase', 'label'});
    writetable(Tbl, fullfile(dataDir, [rid '.csv']));

    % ---- manifest row
    if isFault
        ok = ~isnan(tripT);
        status = ternary(ok, 'ok', 'no_trip_timeout');
        injTs = sprintf('%d', ts0 + round(tinj * 1000));
        trTs = ternary(ok, sprintf('%d', ts0 + round(tripT * 1000)), '');
        trCode = ternary(ok, sprintf('%d', code.(p.kind)), '');
        expCode = sprintf('%d', code.(p.kind));
    else
        status = ternary(isnan(tripT), 'ok', 'unexpected_trip');
        injTs = ''; trTs = ''; trCode = ''; expCode = '';
    end
    sp2 = ternary(isnan(p.speed2), '', sprintf('%d', p.speed2));
    fid = fopen(manifest, 'a');
    fprintf(fid, '%s,%s,%d,%s,%d,%s,%s,%s,%s,%s,0,%d,%d,1\n', rid, p.kind, p.speed, sp2, round(hold_s), ...
        injTs, trTs, trCode, expCode, status, warm, double(isFault));
    fclose(fid);

    nRun = nRun + 1;
    left = sum(~ismember(arrayfun(@(k) sprintf('sim_%03d_%s_%d', k, plan(k).kind, plan(k).speed), r+1:numel(plan), ...
        'UniformOutput', false), done));
    el = toc(t0all);
    fprintf('%3d/%d %-26s %-15s sim %6.0f s in %5.1f s   trip %s   left %d, about %.0f min\n', ...
        r, numel(plan), rid, status, tstop, simSec, ternary(isnan(tripT), '-', sprintf('%.1f s after inj', tripT - tinj)), ...
        left, left * el / nRun / 60);
end
fprintf('\nDone: %d runs in %.1f min. Data in %s\n', nRun, toc(t0all) / 60, dataDir);
end

%% ======================= helpers =======================
function out = ternary(c, a, b)
if c, out = a; else, out = b; end
end

function y = rs(ts, tq)
[tt, ia] = unique(ts.Time, 'last');
d = ts.Data(:);
y = interp1(tt, d(ia), tq, 'linear', 'extrap');
end
