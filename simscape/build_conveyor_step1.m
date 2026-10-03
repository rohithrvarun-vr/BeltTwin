function build_conveyor_step1()
% build_conveyor_step1.m  (BeltTwin option A, step 1)
% Builds conveyor_step1.slx: DC motor -> simple gear (20:1) -> drive drum inertia, with
% Coulomb + viscous friction on the drum, and a PI speed controller that sets the motor
% voltage. Simulates 20 s at a setpoint of 60 BeltTwin speed units and reports the steady
% state against the PLC model's current map  I = 2 A + 0.0875 A per speed unit.
%
% Speed units: BeltTwin speed u (0..80) = belt speed u/100 m/s.
% Motor speed w_m = N * v / r = 20 * (u/100) / 0.1 = 2*u rad/s  (160 rad/s at u = 80).
%
% Everything is written to step1_report.txt as well as the command window.

%% ---------------- parameters ----------------
P.N      = 20;       % gear ratio motor : drum
P.r      = 0.1;      % drum radius, m
P.k      = 0.25;     % torque constant N*m/A = back-EMF constant V/(rad/s)
P.Ra     = 0.5;      % armature resistance, ohm
P.La     = 5e-3;     % armature inductance, H
P.Jm     = 2e-3;     % rotor inertia, kg*m^2
P.Jd     = 0.5;      % drum + belt inertia referred to the drum, kg*m^2
% Friction chosen so that steady-state current matches the PLC map I = 2 + 0.04375*w_m:
%   Coulomb (motor side) = 2 A * k = 0.5 N*m  -> drum side 0.5*N = 10 N*m
%   viscous (motor side) = 0.04375 * k = 0.0109375 N*m/(rad/s) -> drum side * N^2
P.Tc_d   = 0.5 * P.N;
P.b_d    = 0.04375 * P.k * P.N^2;
P.Vmax   = 60;       % supply limit, V
P.u_set  = 60;       % speed setpoint, BeltTwin units
P.u_rate = 50;       % ramp, units per second (as the PLC ramp)
P.Kp     = 0.2;      % PI on motor speed, V/(rad/s)
P.Ki     = 5;
P.Tstop  = 20;

mdl = 'conveyor_step1';
here = fileparts(mfilename('fullpath'));
cd(here);
if exist('step1_report.txt', 'file'), delete('step1_report.txt'); end
diary('step1_report.txt');
cleanup = onCleanup(@() diary('off'));
fprintf('build_conveyor_step1  %s  MATLAB %s\n\n', datestr(now), version);

%% ---------------- model ----------------
libs = {'fl_lib', 'ee_lib', 'sdl_lib', 'nesl_utility', 'simulink'};
for i = 1:numel(libs), load_system(libs{i}); end
if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist([mdl '.slx'], 'file'), delete([mdl '.slx']); end
new_system(mdl);

B = struct();
B.motor = addb(mdl, 'DC Motor',                        'Motor',        [300 200 380 280]);
B.cvs   = addb(mdl, 'Controlled Voltage Source',       'Supply',       [120 200 180 280]);
B.cs    = addb(mdl, 'Current Sensor',                  'CurrentSensor',[200 100 260 160]);
B.eref  = addb(mdl, 'Electrical Reference',            'ERef',         [140 330 180 370]);
B.solv  = addb(mdl, 'Solver Configuration',            'Solver',       [40 330 100 370]);
B.gear  = addb(mdl, 'Simple Gear',                     'Gear',         [460 200 520 260]);
B.drum  = addb(mdl, 'Inertia',                         'Drum',         [600 120 650 170]);
B.fric  = addb(mdl, 'Rotational Friction',             'DrumFriction', [600 220 660 260]);
B.mref1 = addb(mdl, 'Mechanical Rotational Reference', 'MRef1',        [380 330 420 370]);
B.mref2 = addb(mdl, 'Mechanical Rotational Reference', 'MRef2',        [700 300 740 340]);
B.wsens = addb(mdl, 'Ideal Rotational Motion Sensor',  'SpeedSensor',  [420 40 480 100]);
B.mref3 = addb(mdl, 'Mechanical Rotational Reference', 'MRef3',        [520 20 560 60]);
B.sps_v = addb(mdl, 'Simulink-PS Converter',           'V_in',         [60 230 90 250]);
B.pss_w = addb(mdl, 'PS-Simulink Converter',           'W_out',        [540 80 570 100]);
B.pss_i = addb(mdl, 'PS-Simulink Converter',           'I_out',        [300 80 330 100]);

% Simulink control side
add_block('simulink/Sources/Step',                  [mdl '/Setpoint'],  'Position', [-420 -120 -390 -90], ...
    'Time', '1', 'Before', '0', 'After', num2str(P.u_set));
add_block('simulink/Discontinuities/Rate Limiter',  [mdl '/Ramp'],      'Position', [-350 -120 -310 -90], ...
    'RisingSlewLimit', num2str(P.u_rate), 'FallingSlewLimit', num2str(-P.u_rate));
add_block('simulink/Math Operations/Gain',          [mdl '/u_to_w'],    'Position', [-270 -120 -230 -90], ...
    'Gain', num2str(P.N / (100 * P.r)));
add_block('simulink/Math Operations/Sum',           [mdl '/Err'],       'Position', [-190 -115 -170 -95], ...
    'Inputs', '+-');
add_block('simulink/Continuous/PID Controller',     [mdl '/PI'],        'Position', [-140 -125 -90 -85]);
setp(mdl, 'PI', {'Controller'}, 'PI');
setp(mdl, 'PI', {'P'}, num2str(P.Kp));
setp(mdl, 'PI', {'I'}, num2str(P.Ki));
setp(mdl, 'PI', {'LimitOutput'}, 'on');
setp(mdl, 'PI', {'UpperSaturationLimit'}, num2str(P.Vmax));
setp(mdl, 'PI', {'LowerSaturationLimit'}, '0');
setp(mdl, 'PI', {'AntiWindupMode'}, 'clamping');
% voltage driver: first-order lag (1 ms, about 160 Hz bandwidth), like a real PWM amplifier.
% It also breaks the algebraic loop PI -> voltage -> speed -> PI that Simulink reports otherwise.
add_block('simulink/Continuous/Transfer Fcn',       [mdl '/Driver'],    'Position', [-60 -120 -10 -90], ...
    'Numerator', '[1]', 'Denominator', '[1e-3 1]');
add_block('simulink/Math Operations/Gain',          [mdl '/w_to_u'],    'Position', [640 80 680 100], ...
    'Gain', num2str(100 * P.r / P.N));
add_block('simulink/Sinks/To Workspace',            [mdl '/log_u'],     'Position', [720 80 780 100], ...
    'VariableName', 'speed_u', 'SaveFormat', 'Timeseries');
add_block('simulink/Sinks/To Workspace',            [mdl '/log_i'],     'Position', [380 40 440 60], ...
    'VariableName', 'current_A', 'SaveFormat', 'Timeseries');
add_block('simulink/Sinks/To Workspace',            [mdl '/log_v'],     'Position', [-60 -60 0 -40], ...
    'VariableName', 'voltage_V', 'SaveFormat', 'Timeseries');
add_block('simulink/Sinks/To Workspace',            [mdl '/log_set'],   'Position', [-230 -60 -170 -40], ...
    'VariableName', 'setpoint_u', 'SaveFormat', 'Timeseries');

%% ---------------- block parameters ----------------
fprintf('---- block parameter names (for adapting the script) ----\n');
for f = {}
    h = B.(f{1});
    dp = fieldnames(get_param(h, 'DialogParameters'));
    fprintf('%s:\n', get_param(h, 'Name'));
    for j = 1:numel(dp)
        try v = get_param(h, dp{j}); catch, v = '?'; end
        if ~ischar(v), v = '<non-text>'; end
        fprintf('    %-28s = %s\n', dp{j}, strrep(v, newline, ' '));
    end
end
fprintf('\n---- setting parameters ----\n');
% DC motor: equivalent-circuit parameterisation if offered
setp(mdl, 'Motor', {'paramflag', 'electrical_torque_parameterization', 'Parameterization'}, '', true);
setp(mdl, 'Motor', {'R', 'Ra', 'armature_resistance'}, num2str(P.Ra));
setp(mdl, 'Motor', {'L', 'La', 'armature_inductance'}, num2str(P.La));
setp(mdl, 'Motor', {'kv', 'Kv', 'k', 'K', 'back_emf_constant'}, num2str(P.k));
setp(mdl, 'Motor', {'J', 'Jm', 'rotor_inertia', 'inertia'}, num2str(P.Jm));
setp(mdl, 'Motor', {'B', 'Bm', 'rotor_damping', 'damping'}, '0');
% units (only where the block has a separate unit parameter)
setp(mdl, 'Motor', {'R_unit', 'Ra_unit'}, 'Ohm');
setp(mdl, 'Motor', {'L_unit', 'La_unit'}, 'H');
setp(mdl, 'Motor', {'kv_unit', 'Kv_unit', 'k_unit', 'K_unit'}, 'V/(rad/s)');
setp(mdl, 'Motor', {'J_unit', 'Jm_unit'}, 'kg*m^2');
% gear: drum turns N times slower
setp(mdl, 'Gear', {'ratio', 'gear_ratio', 'R', 'FtoB'}, num2str(P.N));
% drum inertia
setp(mdl, 'Drum', {'inertia', 'J'}, num2str(P.Jd));
setp(mdl, 'Drum', {'inertia_unit', 'J_unit'}, 'kg*m^2');
% drum friction (Coulomb + viscous)
setp(mdl, 'DrumFriction', {'brkwy_trq', 'breakaway_torque', 'brkwy_trq_val'}, num2str(1.1 * P.Tc_d));
setp(mdl, 'DrumFriction', {'Col_trq', 'coulomb_torque', 'coulomb_friction_torque'}, num2str(P.Tc_d));
setp(mdl, 'DrumFriction', {'visc_coef', 'viscous_coefficient', 'viscous_friction_coefficient'}, num2str(P.b_d));
setp(mdl, 'DrumFriction', {'brkwy_trq_unit'}, 'N*m');
setp(mdl, 'DrumFriction', {'Col_trq_unit'}, 'N*m');
setp(mdl, 'DrumFriction', {'visc_coef_unit'}, 'N*m*s/rad');

%% ---------------- physical connections ----------------
% Ports are picked by where they sit on the block icon, not by LConn/RConn index:
% the index order differs between blocks whose ports sit on the top/bottom edges.
% Labels as seen on the icons (port_probe screenshot):
%   DC Motor       top: + , R        bottom: - , C      (left to right)
%   Voltage source top: +            bottom: V(signal) , -
%   Current sensor left: +           right: I(signal) , -   (top to bottom)
%   Simple Gear    left: B           right: F
%   Friction       left: R           right: C
%   Speed sensor   left: R           right: C , W , A
fprintf('\n---- port geometry (for checking) ----\n');
dumpports(B.motor); dumpports(B.cvs); dumpports(B.cs); dumpports(B.wsens);
% Resolve every port handle BEFORE drawing any line: drawing lines can shift port
% positions on the icon, which would confuse the edge lookup for later ports.
T.sup_p   = pp(B.cvs, 'top', 1);       T.sup_V  = pp(B.cvs, 'bottom', 1);   T.sup_n = pp(B.cvs, 'bottom', 2);
T.cs_p    = pp(B.cs, 'left', 1);       T.cs_I   = pp(B.cs, 'right', 1);     T.cs_n  = pp(B.cs, 'right', 2);
T.mot_p   = pp(B.motor, 'top', 1);     T.mot_R  = pp(B.motor, 'top', 2);
T.mot_n   = pp(B.motor, 'bottom', 1);  T.mot_C  = pp(B.motor, 'bottom', 2);
T.gear_B  = pp(B.gear, 'left', 1);     T.gear_F = pp(B.gear, 'right', 1);
T.fric_R  = pp(B.fric, 'left', 1);     T.fric_C = pp(B.fric, 'right', 1);
T.ws_R    = pp(B.wsens, 'left', 1);    T.ws_C   = pp(B.wsens, 'right', 1);  T.ws_W  = pp(B.wsens, 'right', 2);
T.eref    = pp(B.eref, 'only', 1);     T.solv   = pp(B.solv, 'only', 1);
T.drum    = pp(B.drum, 'only', 1);
T.mref1   = pp(B.mref1, 'only', 1);    T.mref2  = pp(B.mref2, 'only', 1);   T.mref3 = pp(B.mref3, 'only', 1);
T.sps_v   = pp(B.sps_v, 'only', 1);    T.pss_i  = pp(B.pss_i, 'only', 1);   T.pss_w = pp(B.pss_w, 'only', 1);
% electrical: Supply + -> CurrentSensor + ; CurrentSensor - -> Motor + ; Motor - -> Supply - -> ERef
conn(mdl, T.sup_p, T.cs_p);
conn(mdl, T.cs_n,  T.mot_p);
conn(mdl, T.mot_n, T.sup_n);
conn(mdl, T.sup_n, T.eref);
conn(mdl, T.solv,  T.eref);
conn(mdl, T.sps_v, T.sup_V);
conn(mdl, T.cs_I,  T.pss_i);
% mechanical: Motor R -> Gear B ; Motor C -> ref ; Gear F -> Drum, Friction R ; Friction C -> ref
conn(mdl, T.mot_R,  T.gear_B);
conn(mdl, T.mot_C,  T.mref1);
conn(mdl, T.gear_F, T.drum);
conn(mdl, T.gear_F, T.fric_R);
conn(mdl, T.fric_C, T.mref2);
% speed sensor on the motor shaft: R -> Motor R, C -> ref, W -> PS-S
conn(mdl, T.ws_R, T.mot_R);
conn(mdl, T.ws_C, T.mref3);
conn(mdl, T.ws_W, T.pss_w);

%% ---------------- signal connections ----------------
sl(mdl, 'Setpoint/1', 'Ramp/1');
sl(mdl, 'Ramp/1', 'u_to_w/1');
sl(mdl, 'Ramp/1', 'log_set/1');
sl(mdl, 'u_to_w/1', 'Err/1');
sl(mdl, 'W_out/1', 'Err/2');
sl(mdl, 'Err/1', 'PI/1');
sl(mdl, 'PI/1', 'Driver/1');
sl(mdl, 'Driver/1', 'V_in/1');
sl(mdl, 'Driver/1', 'log_v/1');
sl(mdl, 'W_out/1', 'w_to_u/1');
sl(mdl, 'w_to_u/1', 'log_u/1');
sl(mdl, 'I_out/1', 'log_i/1');

set_param(mdl, 'StopTime', num2str(P.Tstop), 'Solver', 'ode23t', 'RelTol', '1e-4', 'MaxStep', '1e-2');
save_system(mdl);
fprintf('\nModel %s.slx built and saved.\n', mdl);

%% ---------------- simulate and report ----------------
out = sim(mdl);
u  = out.get('speed_u');  i_ = out.get('current_A');  v = out.get('voltage_V');
t  = u.Time;  ss = t >= P.Tstop - 5;
u_ss = mean(u.Data(ss));  i_ss = mean(i_.Data(ss));  v_ss = mean(v.Data(ss));
i_plc = 2 + 0.0875 * P.u_set;
fprintf('\n---- steady state (last 5 s) ----\n');
fprintf('speed      %7.2f units   (setpoint %g)\n', u_ss, P.u_set);
fprintf('current    %7.2f A       (PLC map %.2f A)\n', i_ss, i_plc);
fprintf('voltage    %7.2f V       (limit %g V)\n', v_ss, P.Vmax);
if u_ss < 0, fprintf('WARNING: speed is negative: a port or sign is reversed.\n'); end
if abs(u_ss - P.u_set) > 1, fprintf('WARNING: speed does not reach the setpoint.\n'); end

figure('Name', 'conveyor_step1');
subplot(3,1,1); plot(t, u.Data, out.get('setpoint_u').Time, out.get('setpoint_u').Data, '--');
ylabel('speed (units)'); legend('measured', 'setpoint', 'Location', 'southeast'); grid on;
subplot(3,1,2); plot(i_.Time, i_.Data); ylabel('current (A)'); grid on;
yline(i_plc, ':', 'PLC map');
subplot(3,1,3); plot(v.Time, v.Data); ylabel('voltage (V)'); xlabel('time (s)'); grid on;
fprintf('\nDone. Send step1_report.txt and a screenshot of the figure.\n');
end

%% ======================= helpers =======================
function h = addb(mdl, libname, name, pos)
% find a library block by name (whitespace and line breaks ignored) and add it
persistent cache
if isempty(cache), cache = containers.Map(); end
key = lower(regexprep(libname, '\s+', ' '));
if isKey(cache, key)
    path = cache(key);
else
    path = '';
    for lib = {'fl_lib', 'ee_lib', 'sdl_lib', 'nesl_utility'}
        try
            c = find_system(lib{1}, 'SearchDepth', 3, 'LookUnderMasks', 'all', 'FollowLinks', 'on', ...
                'MatchFilter', @Simulink.match.allVariants, 'Type', 'block');
        catch
            c = find_system(lib{1}, 'SearchDepth', 3, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'Type', 'block');
        end
        for k = 1:numel(c)
            if strcmp(lower(strtrim(regexprep(get_param(c{k}, 'Name'), '\s+', ' '))), key)
                if isempty(path) || numel(c{k}) < numel(path), path = c{k}; end
            end
        end
        if ~isempty(path), break; end
    end
    if isempty(path), error('Library block not found: %s', libname); end
    cache(key) = path;
end
h = add_block(path, [mdl '/' name], 'Position', pos);
end

function [edges, xy, hs, tags] = portedges(h)
% physical ports of a block with the icon edge each sits on
ph = get_param(h, 'PortHandles');
hs = [ph.LConn(:); ph.RConn(:)];
tags = [arrayfun(@(k) sprintf('LConn%d', k), 1:numel(ph.LConn), 'UniformOutput', false), ...
        arrayfun(@(k) sprintf('RConn%d', k), 1:numel(ph.RConn), 'UniformOutput', false)]';
bp = get_param(h, 'Position');
names = {'left', 'top', 'right', 'bottom'};
edges = cell(numel(hs), 1); xy = zeros(numel(hs), 2);
for k = 1:numel(hs)
    pos = get_param(hs(k), 'Position');
    d = [abs(pos(1) - bp(1)), abs(pos(2) - bp(2)), abs(pos(1) - bp(3)), abs(pos(2) - bp(4))];
    [~, e] = min(d);
    edges{k} = names{e}; xy(k, :) = pos(1:2);
end
end

function p = pp(h, edge, k)
% k-th physical port on an icon edge: top/bottom ordered left to right, left/right top to bottom
[edges, xy, hs] = portedges(h);
if strcmp(edge, 'only')
    if numel(hs) ~= 1, error('%s has %d physical ports, expected 1', get_param(h, 'Name'), numel(hs)); end
    p = hs(1); return;
end
sel = find(strcmp(edges, edge));
if any(strcmp(edge, {'top', 'bottom'})), [~, o] = sort(xy(sel, 1)); else, [~, o] = sort(xy(sel, 2)); end
sel = sel(o);
if k > numel(sel)
    error('%s: no port %d on edge %s (edges found: %s)', get_param(h, 'Name'), k, edge, strjoin(edges', ', '));
end
p = hs(sel(k));
end

function dumpports(h)
[edges, xy, ~, tags] = portedges(h);
bp = get_param(h, 'Position');
fprintf('%s  block [%d %d %d %d]\n', get_param(h, 'Name'), bp);
for k = 1:numel(edges)
    fprintf('    %-7s %-7s at (%d, %d)\n', tags{k}, edges{k}, xy(k, 1), xy(k, 2));
end
end

function conn(mdl, p1, p2)
add_line(mdl, p1, p2, 'autorouting', 'on');
end

function sl(mdl, a, b)
add_line(mdl, a, b, 'autorouting', 'on');
end

function setp(mdl, blk, aliases, value, listOnly)
% set the first parameter name in aliases that exists on the block; report the outcome
if nargin < 5, listOnly = false; end
h = [mdl '/' blk];
dp = fieldnames(get_param(h, 'DialogParameters'));
for a = aliases
    if any(strcmp(dp, a{1}))
        if listOnly
            fprintf('%-14s %-34s current value: %s\n', blk, a{1}, strrep(char(get_param(h, a{1})), newline, ' '));
            return;
        end
        try
            set_param(h, a{1}, value);
            fprintf('%-14s %-34s = %s\n', blk, a{1}, value);
        catch err
            fprintf('%-14s %-34s FAILED: %s\n', blk, a{1}, err.message);
        end
        return;
    end
end
fprintf('%-14s NOT SET (none of: %s)\n', blk, strjoin(aliases, ', '));
end
