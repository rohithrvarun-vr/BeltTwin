function build_conveyor_step2()
% build_conveyor_step2.m  (BeltTwin option A, step 2)
% Step 1 plus: the belt as its own part, driven through a friction contact, and a
% randomly varying material load.
%
%   DC motor -> gear 20:1 -> drum (inertia + bearing friction)
%            -> friction clutch (drum-belt grip; can slip)
%            -> wheel and axle (r = 0.1 m) -> belt mass + roller friction + material load
%
% The friction is split between drum bearing and belt rollers, and the mean material
% load is 30 N, so the steady-state current still matches the PLC map
% (I = 2 A + 0.0875 A per speed unit, 7.25 A at speed 60).
% The load varies like the PLC model's unmeasured load: two random first-order
% (Ornstein-Uhlenbeck) components, time constants 20 s and 600 s, 12 N each.
% Belt ratio = belt speed / drum surface speed; it drops below 1 when the contact slips.
%
% Writes step2_report.txt; shows a figure.

%% ---------------- parameters ----------------
P.N      = 20;       % gear ratio motor : drum
P.r      = 0.1;      % drum radius, m
P.k      = 0.25;     % torque / back-EMF constant
P.Ra     = 0.5;  P.La = 5e-3;  P.Jm = 2e-3;
P.Jd     = 0.1;      % drum only, kg*m^2
P.mb     = 100;      % belt + material, kg
% friction split (drum side totals as in step 1: 10 N*m Coulomb, 4.375 N*m*s/rad viscous)
P.Tc_brg = 4;        % drum bearing Coulomb, N*m
P.b_brg  = 1.875;    % drum bearing viscous, N*m*s/rad
P.Fc_rol = 30;       % belt rollers Coulomb, N   (x r = 3 N*m)
P.b_rol  = 250;      % belt rollers viscous, N*s/m (x r^2 = 2.5 N*m*s/rad)
P.F_load = 30;       % mean material load, N     (x r = 3 N*m)
P.sig_f  = 12;  P.tau_f = 20;     % fast load component: sigma N, time constant s
P.sig_s  = 12;  P.tau_s = 600;    % slow load component
P.seed   = 1;
P.loadSign = -1;     % checked in the first run: +1 made the load push the belt forward (correlation -1.00)
% drum-belt grip (clutch torque capacity at the drum), about 2x the need at speed 80
P.T_static  = 60;    % N*m
P.T_kinetic = 50;    % N*m
P.Vmax   = 60;  P.u_set = 60;  P.u_rate = 50;  P.Kp = 0.2;  P.Ki = 5;
P.Tstop  = 300;

mdl = 'conveyor_step2';
here = fileparts(mfilename('fullpath'));
cd(here);
if exist('step2_report.txt', 'file'), delete('step2_report.txt'); end
diary('step2_report.txt');
cleanup = onCleanup(@() diary('off'));
fprintf('build_conveyor_step2  %s  MATLAB %s\n\n', datestr(now), version);

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
B.brg   = addb(mdl, 'Rotational Friction',             'DrumBearing',  [600 300 660 340]);
B.clu   = addb(mdl, 'Fundamental Friction Clutch',     'Grip',         [720 180 820 280]);
B.wa    = addb(mdl, 'Wheel and Axle',                  'Wheel',        [900 200 980 260]);
B.belt  = addb(mdl, 'Mass',                            'Belt',         [1060 80 1110 130]);
B.rol   = addb(mdl, 'Translational Friction',          'Rollers',      [1060 200 1120 240]);
B.fsrc  = addb(mdl, 'Ideal Force Source',              'Load',         [1060 300 1120 360]);
B.tsen  = addb(mdl, 'Ideal Translational Motion Sensor','BeltSensor',  [1200 180 1260 240]);
B.tref1 = addb(mdl, 'Mechanical Translational Reference','TRef1',      [1180 260 1220 300]);
B.tref2 = addb(mdl, 'Mechanical Translational Reference','TRef2',      [1180 380 1220 420]);
B.tref3 = addb(mdl, 'Mechanical Translational Reference','TRef3',      [1300 260 1340 300]);
B.mref1 = addb(mdl, 'Mechanical Rotational Reference', 'MRef1',        [380 330 420 370]);
B.mref2 = addb(mdl, 'Mechanical Rotational Reference', 'MRef2',        [700 360 740 400]);
B.wsens = addb(mdl, 'Ideal Rotational Motion Sensor',  'SpeedSensor',  [420 40 480 100]);
B.mref3 = addb(mdl, 'Mechanical Rotational Reference', 'MRef3',        [520 20 560 60]);
B.sps_v = addb(mdl, 'Simulink-PS Converter',           'V_in',         [60 230 90 250]);
B.sps_f = addb(mdl, 'Simulink-PS Converter',           'F_in',         [980 380 1010 400]);
B.sps_k = addb(mdl, 'Simulink-PS Converter',           'Tk_in',        [640 180 670 200]);
B.sps_p = addb(mdl, 'Simulink-PS Converter',           'Tp_in',        [640 210 670 230]);
B.sps_n = addb(mdl, 'Simulink-PS Converter',           'Tn_in',        [640 240 670 260]);
B.pss_w = addb(mdl, 'PS-Simulink Converter',           'W_out',        [540 80 570 100]);
B.pss_i = addb(mdl, 'PS-Simulink Converter',           'I_out',        [300 80 330 100]);
B.pss_v = addb(mdl, 'PS-Simulink Converter',           'Vbelt_out',    [1300 200 1330 220]);
B.pss_m = addb(mdl, 'PS-Simulink Converter',           'Mode_out',     [860 120 890 140]);

% control side (as step 1)
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
setp(mdl, 'PI', {'ZeroCross'}, 'off');   % no event detection on the clamping switch (it chattered at saturation)
add_block('simulink/Continuous/Transfer Fcn',       [mdl '/Driver'],    'Position', [-60 -120 -10 -90], ...
    'Numerator', '[1]', 'Denominator', '[1e-3 1]');
add_block('simulink/Math Operations/Gain',          [mdl '/w_to_u'],    'Position', [640 80 680 100], ...
    'Gain', num2str(100 * P.r / P.N));

% material load: 30 N + two random first-order components, never negative, opposing motion
% Band-Limited White Noise power c gives a first-order (tau) output variance c/(2 tau).
add_block('simulink/Sources/Band-Limited White Noise', [mdl '/NoiseFast'], 'Position', [600 520 640 550], ...
    'Cov', num2str(2 * P.tau_f * P.sig_f^2), 'Ts', '0.1', 'seed', num2str(P.seed));
add_block('simulink/Sources/Band-Limited White Noise', [mdl '/NoiseSlow'], 'Position', [600 590 640 620], ...
    'Cov', num2str(2 * P.tau_s * P.sig_s^2), 'Ts', '0.1', 'seed', num2str(P.seed + 1000));
add_block('simulink/Continuous/Transfer Fcn', [mdl '/OUfast'], 'Position', [680 515 760 555], ...
    'Numerator', '[1]', 'Denominator', sprintf('[%g 1]', P.tau_f));
add_block('simulink/Continuous/Transfer Fcn', [mdl '/OUslow'], 'Position', [680 585 760 625], ...
    'Numerator', '[1]', 'Denominator', sprintf('[%g 1]', P.tau_s));
add_block('simulink/Sources/Constant', [mdl '/LoadMean'], 'Position', [680 460 720 480], 'Value', num2str(P.F_load));
add_block('simulink/Math Operations/Sum', [mdl '/LoadSum'], 'Position', [800 520 820 580], 'Inputs', '+++');
add_block('simulink/Discontinuities/Saturation', [mdl '/LoadNonNeg'], 'Position', [850 535 890 565], ...
    'UpperLimit', 'inf', 'LowerLimit', '0');
add_block('simulink/Math Operations/Gain', [mdl '/v_scale'], 'Position', [1360 260 1400 280], 'Gain', '100');
add_block('simulink/Math Operations/Trigonometric Function', [mdl '/Dir'], 'Position', [1420 260 1460 280], ...
    'Operator', 'tanh');
add_block('simulink/Math Operations/Product', [mdl '/LoadDir'], 'Position', [920 530 950 570]);
add_block('simulink/Math Operations/Gain', [mdl '/LoadSign'], 'Position', [960 535 990 565], 'Gain', num2str(P.loadSign));
% 50 ms lag between measured belt speed and the load force direction: a real Simulink state,
% which breaks the loop belt speed -> load direction -> force -> belt speed
add_block('simulink/Continuous/Transfer Fcn', [mdl '/LoadLag'], 'Position', [1000 535 1060 565], ...
    'Numerator', '[1]', 'Denominator', '[0.05 1]');
% drum-belt grip capacity
add_block('simulink/Sources/Constant', [mdl '/Tkin'],  'Position', [560 180 600 200], 'Value', num2str(P.T_kinetic));
add_block('simulink/Sources/Constant', [mdl '/Tpos'],  'Position', [560 210 600 230], 'Value', num2str(P.T_static));
add_block('simulink/Sources/Constant', [mdl '/Tneg'],  'Position', [560 240 600 260], 'Value', num2str(-P.T_static));

% logging
logs = {'speed_u', 'current_A', 'voltage_V', 'setpoint_u', 'belt_u', 'load_N', 'grip_mode'};
for q = 1:numel(logs)
    add_block('simulink/Sinks/To Workspace', [mdl '/log_' logs{q}], 'Position', [1500 40+q*40 1560 60+q*40], ...
        'VariableName', logs{q}, 'SaveFormat', 'Timeseries');
end

%% ---------------- block parameters ----------------
fprintf('---- setting parameters ----\n');
setp(mdl, 'Motor', {'Ra'}, num2str(P.Ra));      setp(mdl, 'Motor', {'Ra_unit'}, 'Ohm');
setp(mdl, 'Motor', {'La'}, num2str(P.La));      setp(mdl, 'Motor', {'La_unit'}, 'H');
setp(mdl, 'Motor', {'Kv'}, num2str(P.k));       setp(mdl, 'Motor', {'Kv_unit'}, 'V/(rad/s)');
setp(mdl, 'Motor', {'J'}, num2str(P.Jm));       setp(mdl, 'Motor', {'J_unit'}, 'kg*m^2');
setp(mdl, 'Gear', {'ratio'}, num2str(P.N));
setp(mdl, 'Drum', {'inertia'}, num2str(P.Jd));  setp(mdl, 'Drum', {'inertia_unit'}, 'kg*m^2');
setp(mdl, 'DrumBearing', {'brkwy_trq'}, num2str(1.1 * P.Tc_brg));
setp(mdl, 'DrumBearing', {'Col_trq'}, num2str(P.Tc_brg));
setp(mdl, 'DrumBearing', {'visc_coef'}, num2str(P.b_brg));
setp(mdl, 'Wheel', {'R'}, num2str(P.r));        setp(mdl, 'Wheel', {'R_unit'}, 'm');
setp(mdl, 'Belt', {'mass'}, num2str(P.mb));     setp(mdl, 'Belt', {'mass_unit'}, 'kg');
setp(mdl, 'Rollers', {'brkwy_frc'}, num2str(1.1 * P.Fc_rol));
setp(mdl, 'Rollers', {'brkwy_vel'}, '0.01');
setp(mdl, 'Rollers', {'Col_frc'}, num2str(P.Fc_rol));
setp(mdl, 'Rollers', {'visc_coef'}, num2str(P.b_rol));
setp(mdl, 'Grip', {'initial_state_locked'}, 'sdl.enum.initial_lock.locked');
for c = {'Tk_in', 'Tp_in', 'Tn_in'}
    setp(mdl, c{1}, {'Unit'}, 'N*m');
end
setp(mdl, 'F_in', {'Unit'}, 'N');
% the force acts on the belt mass, so Simscape needs the force's time derivative:
% let the converter filter the signal (first order, 10 ms) and compute it.
spsp = fieldnames(get_param([mdl '/F_in'], 'DialogParameters'));
fprintf('F_in converter parameters: %s\n', strjoin(spsp', ', '));
setp(mdl, 'F_in', {'FilteringAndDerivatives', 'FilterAndDerivatives', 'InputFiltering'}, 'filter');
setp(mdl, 'F_in', {'InputFilterOrder', 'FilterOrder'}, '1');
setp(mdl, 'F_in', {'InputFilterTimeConstant', 'FilterTimeConstant'}, '0.01');
setp(mdl, 'Vbelt_out', {'Unit'}, 'm/s');

%% ---------------- physical connections ----------------
fprintf('\n---- port geometry (for checking) ----\n');
dumpports(B.clu); dumpports(B.wa); dumpports(B.fsrc); dumpports(B.tsen);
% resolve all port handles before drawing lines
T.sup_p = pp(B.cvs, 'top', 1);      T.sup_V = pp(B.cvs, 'bottom', 1);   T.sup_n = pp(B.cvs, 'bottom', 2);
T.cs_p  = pp(B.cs, 'left', 1);      T.cs_I  = pp(B.cs, 'right', 1);     T.cs_n  = pp(B.cs, 'right', 2);
T.mot_p = pp(B.motor, 'top', 1);    T.mot_R = pp(B.motor, 'top', 2);
T.mot_n = pp(B.motor, 'bottom', 1); T.mot_C = pp(B.motor, 'bottom', 2);
T.gear_B = pp(B.gear, 'left', 1);   T.gear_F = pp(B.gear, 'right', 1);
T.brg_R = pp(B.brg, 'left', 1);     T.brg_C = pp(B.brg, 'right', 1);
% clutch: left tK, t+, t-, B (top to bottom); right S, M, F
T.clu_tk = pp(B.clu, 'left', 1);    T.clu_tp = pp(B.clu, 'left', 2);    T.clu_tn = pp(B.clu, 'left', 3);
T.clu_B  = pp(B.clu, 'left', 4);    T.clu_M  = pp(B.clu, 'right', 2);   T.clu_F  = pp(B.clu, 'right', 3);
T.wa_A  = pp(B.wa, 'left', 1);      T.wa_P  = pp(B.wa, 'right', 1);
T.belt  = pp(B.belt, 'only', 1);
T.rol_R = pp(B.rol, 'left', 1);     T.rol_C = pp(B.rol, 'right', 1);
% force source: top R; bottom S (signal), C
T.f_R   = pp(B.fsrc, 'top', 1);     T.f_S   = pp(B.fsrc, 'bottom', 1);  T.f_C   = pp(B.fsrc, 'bottom', 2);
% translational sensor: left R; right C, V, P
T.ts_R  = pp(B.tsen, 'left', 1);    T.ts_C  = pp(B.tsen, 'right', 1);   T.ts_V  = pp(B.tsen, 'right', 2);
T.ws_R  = pp(B.wsens, 'left', 1);   T.ws_C  = pp(B.wsens, 'right', 1);  T.ws_W  = pp(B.wsens, 'right', 2);
T.eref  = pp(B.eref, 'only', 1);    T.solv  = pp(B.solv, 'only', 1);    T.drum  = pp(B.drum, 'only', 1);
T.mref1 = pp(B.mref1, 'only', 1);   T.mref2 = pp(B.mref2, 'only', 1);   T.mref3 = pp(B.mref3, 'only', 1);
T.tref1 = pp(B.tref1, 'only', 1);   T.tref2 = pp(B.tref2, 'only', 1);   T.tref3 = pp(B.tref3, 'only', 1);
T.sps_v = pp(B.sps_v, 'only', 1);   T.sps_f = pp(B.sps_f, 'only', 1);
T.sps_k = pp(B.sps_k, 'only', 1);   T.sps_p = pp(B.sps_p, 'only', 1);   T.sps_n = pp(B.sps_n, 'only', 1);
T.pss_i = pp(B.pss_i, 'only', 1);   T.pss_w = pp(B.pss_w, 'only', 1);
T.pss_v = pp(B.pss_v, 'only', 1);   T.pss_m = pp(B.pss_m, 'only', 1);
% electrical
conn(mdl, T.sup_p, T.cs_p);   conn(mdl, T.cs_n, T.mot_p);   conn(mdl, T.mot_n, T.sup_n);
conn(mdl, T.sup_n, T.eref);   conn(mdl, T.solv, T.eref);    conn(mdl, T.sps_v, T.sup_V);
conn(mdl, T.cs_I, T.pss_i);
% motor -> gear -> drum (+ bearing) -> grip
conn(mdl, T.mot_R, T.gear_B); conn(mdl, T.mot_C, T.mref1);
conn(mdl, T.gear_F, T.drum);  conn(mdl, T.gear_F, T.brg_R); conn(mdl, T.brg_C, T.mref2);
conn(mdl, T.gear_F, T.clu_B);
conn(mdl, T.sps_k, T.clu_tk); conn(mdl, T.sps_p, T.clu_tp); conn(mdl, T.sps_n, T.clu_tn);
conn(mdl, T.clu_M, T.pss_m);
% grip -> wheel -> belt node: mass, rollers, load, sensor
conn(mdl, T.clu_F, T.wa_A);
conn(mdl, T.wa_P, T.belt);    conn(mdl, T.wa_P, T.rol_R);   conn(mdl, T.wa_P, T.f_R);
conn(mdl, T.wa_P, T.ts_R);
conn(mdl, T.rol_C, T.tref1);  conn(mdl, T.f_C, T.tref2);    conn(mdl, T.ts_C, T.tref3);
conn(mdl, T.sps_f, T.f_S);    conn(mdl, T.ts_V, T.pss_v);
% motor speed sensor
conn(mdl, T.ws_R, T.mot_R);   conn(mdl, T.ws_C, T.mref3);   conn(mdl, T.ws_W, T.pss_w);

%% ---------------- signal connections ----------------
sl(mdl, 'Setpoint/1', 'Ramp/1');    sl(mdl, 'Ramp/1', 'u_to_w/1');   sl(mdl, 'Ramp/1', 'log_setpoint_u/1');
sl(mdl, 'u_to_w/1', 'Err/1');       sl(mdl, 'W_out/1', 'Err/2');     sl(mdl, 'Err/1', 'PI/1');
sl(mdl, 'PI/1', 'Driver/1');        sl(mdl, 'Driver/1', 'V_in/1');   sl(mdl, 'Driver/1', 'log_voltage_V/1');
sl(mdl, 'W_out/1', 'w_to_u/1');     sl(mdl, 'w_to_u/1', 'log_speed_u/1');
sl(mdl, 'I_out/1', 'log_current_A/1');
sl(mdl, 'NoiseFast/1', 'OUfast/1'); sl(mdl, 'NoiseSlow/1', 'OUslow/1');
sl(mdl, 'LoadMean/1', 'LoadSum/1'); sl(mdl, 'OUfast/1', 'LoadSum/2'); sl(mdl, 'OUslow/1', 'LoadSum/3');
sl(mdl, 'LoadSum/1', 'LoadNonNeg/1');
sl(mdl, 'LoadNonNeg/1', 'LoadDir/1'); sl(mdl, 'LoadNonNeg/1', 'log_load_N/1');
sl(mdl, 'Vbelt_out/1', 'v_scale/1'); sl(mdl, 'v_scale/1', 'Dir/1');  sl(mdl, 'Dir/1', 'LoadDir/2');
sl(mdl, 'LoadDir/1', 'LoadSign/1'); sl(mdl, 'LoadSign/1', 'LoadLag/1'); sl(mdl, 'LoadLag/1', 'F_in/1');
add_block('simulink/Math Operations/Gain', [mdl '/belt_to_u'], 'Position', [1360 200 1400 220], 'Gain', '100');
sl(mdl, 'Vbelt_out/1', 'belt_to_u/1'); sl(mdl, 'belt_to_u/1', 'log_belt_u/1');
sl(mdl, 'Mode_out/1', 'log_grip_mode/1');
sl(mdl, 'Tkin/1', 'Tk_in/1');       sl(mdl, 'Tpos/1', 'Tp_in/1');     sl(mdl, 'Tneg/1', 'Tn_in/1');

set_param(mdl, 'StopTime', num2str(P.Tstop), 'Solver', 'ode23t', 'RelTol', '1e-4', 'MaxStep', '0.05', ...
    'ZeroCrossAlgorithm', 'Adaptive', 'IgnoredZcDiagnostic', 'none');
save_system(mdl);
fprintf('\nModel %s.slx built and saved.\n', mdl);

%% ---------------- simulate and report ----------------
out = sim(mdl);
u  = out.get('speed_u');  i_ = out.get('current_A');  v = out.get('voltage_V');
ub = out.get('belt_u');   fl = out.get('load_N');     gm = out.get('grip_mode');
t  = u.Time;  ss = t >= 30;
ubi = rs(ub, t(ss));            % all signals on the motor-speed time grid
ii  = rs(i_, t(ss));
fli = rs(fl, t(ss));
vi  = rs(v, t(ss));
ratio = abs(ubi) ./ max(abs(u.Data(ss)), 1e-6);
fprintf('\n---- after warm-up (t >= 30 s) ----\n');
fprintf('motor-side speed  mean %6.2f units (setpoint %g)\n', mean(u.Data(ss)), P.u_set);
fprintf('belt speed        mean %6.2f units\n', mean(abs(ubi)));
fprintf('belt ratio        mean %.4f   min %.4f   (1 = no slip)\n', mean(ratio), min(ratio));
fprintf('current           mean %6.2f A   std %.2f A   (PLC map %.2f A)\n', mean(ii), std(ii), 2 + 0.0875 * P.u_set);
fprintf('load force        mean %6.1f N   std %.1f N\n', mean(fli), std(fli));
fprintf('voltage           mean %6.2f V\n', mean(vi));
fprintf('grip mode values seen: %s\n', mat2str(unique(round(gm.Data(:)))'));
cl = corrcoef(fli, ii);
fprintf('correlation load vs current: %.2f  (should be clearly positive; negative = load sign reversed)\n', cl(1, 2));
if cl(1, 2) < 0, fprintf('WARNING: set P.loadSign = -1 and run again.\n'); end

figure('Name', 'conveyor_step2');
subplot(4,1,1); plot(t, u.Data, ub.Time, abs(ub.Data), '--'); ylabel('speed (units)');
legend('drum surface (from motor)', 'belt', 'Location', 'southeast'); grid on;
subplot(4,1,2); plot(i_.Time, i_.Data); ylabel('current (A)'); grid on; yline(2 + 0.0875 * P.u_set, ':', 'PLC map');
subplot(4,1,3); plot(fl.Time, fl.Data); ylabel('load (N)'); grid on;
subplot(4,1,4); plot(t(ss), ratio); ylabel('belt ratio'); xlabel('time (s)'); grid on;
fprintf('\nDone. Send step2_report.txt and a screenshot of the figure.\n');
end

%% ======================= helpers =======================
function y = rs(ts, tq)
% resample a logged timeseries onto query times (logs can repeat a time at events)
[tt, ia] = unique(ts.Time, 'last');
d = ts.Data(:);
y = interp1(tt, d(ia), tq, 'linear', 'extrap');
end

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
