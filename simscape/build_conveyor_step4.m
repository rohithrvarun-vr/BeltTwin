function build_conveyor_step4(fault)
% build_conveyor_step4.m  (BeltTwin option A, step 4)
% Usage: build_conveyor_step4('jam' | 'slip' | 'overload' | 'wear' | 'none')
% Step 3 plus one fault, injected after a 300 s warm-up. A hidden severity s grows from 0
% (jam linear, the others quadratic: slow start), as in the PLC's sensor model v3; at s = 0.8
% the conveyor trips (motor voltage cut). Only a physical part changes; Simscape computes
% what the sensors see:
%   jam       extra braking force on the belt, up to 1500 N  (belt blocks, drum slips on it)
%   slip      drum-belt grip capacity falls to (1 - s) of nominal
%   overload  extra material load, up to 300 N  (= +6 A at s = 1, as in the PLC model)
%   wear      extra bearing friction torque, up to 2.4 N*m  (bearing about +22 K at trip, as in
%             the PLC model; the motor supplies that power, so the current rises slightly, ~0.4 A)
% Step 3 notes: temperatures:
%   motor winding  heated by copper loss I^2*R inside the DC Motor's own thermal model
%                  (winding resistance rises with temperature), cooled by convection to ambient
%   drum bearing   heated by its friction loss (friction torque x drum speed), measured with a
%                  torque sensor; thermal mass + convection to ambient
%   ambient        slow random drift around 22 degC (sigma 1 degC, time constant 1 h)
% Thermal values are calibrated so that at speed 60 the steady rises match the PLC model
% (motor +30 degC, bearing +10.5 degC, time constants 60 s and 120 s). At other speeds the
% physics differs on purpose (copper loss grows with current squared).
% Step 2 notes:
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
% Writes step4_<fault>_report.txt; shows a figure.

%% ---------------- parameters ----------------
P.N      = 20;       % gear ratio motor : drum
P.r      = 0.1;      % drum radius, m
P.k      = 0.25;     % torque / back-EMF constant
P.Ra     = 0.5;  P.La = 5e-3;  P.Jm = 2e-3;
P.Jd     = 0.1;      % drum only, kg*m^2
P.mb     = 100;      % belt + material, kg
% friction split (drum side totals as in step 1: 10 N*m Coulomb, 4.375 N*m*s/rad viscous)
% drum friction split: only a small part is the bearing itself (and heats it);
% the rest is seals/scrapers (no heat path modelled). Totals unchanged: 4 N*m, 1.875 N*m*s/rad.
P.Tc_brg  = 0.3;     % drum bearing Coulomb, N*m       (heats the bearing)
P.b_brg   = 0.1;     % drum bearing viscous, N*m*s/rad (bearing loss ~5.4 W at speed 60)
P.Tc_seal = 3.7;     % seals/scrapers Coulomb, N*m
P.b_seal  = 1.775;   % seals/scrapers viscous, N*m*s/rad
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
if nargin < 1, fault = 'jam'; end
fault = lower(fault);
P.t_inj  = 300;                                   % s, after thermal warm-up
P.T_nom  = struct('jam', 2, 'slip', 25, 'overload', 50, 'wear', 400, 'none', 100);
P.quad   = struct('jam', false, 'slip', true, 'overload', true, 'wear', true, 'none', true);
if ~isfield(P.T_nom, fault), error('fault must be jam, slip, overload, wear or none'); end
P.T      = P.T_nom.(fault);
P.F_jam  = 1500;  P.F_ov = 300;  P.T_wear = 2.4;   % wear: +11.5 W in the bearing at s = 0.8 -> about +22 K, +0.4 A
P.wearSign = 1;   % flip to -1 if the report says wear lowers the current
P.Tstop  = P.t_inj + 1.3 * P.T + 30;
% thermal
P.T_amb0  = 22;       % degC
P.sig_amb = 1;   P.tau_amb = 3600;
P.hA_mot  = 0.97;     % W/K   (rise 30 K from ~29 W copper loss at speed 60, hot winding)
P.C_mot   = 58;       % J/K   (time constant C/hA = 60 s)
P.hA_brg  = 0.514;    % W/K   (rise 10.5 K from ~5.4 W bearing loss at speed 60)
P.C_brg   = 61.7;     % J/K   (time constant 120 s)
P.qSign   = 1;        % flip to -1 if the report says the bearing cools below ambient

mdl = 'conveyor_step4';
here = fileparts(mfilename('fullpath'));
cd(here);
rep = sprintf('step4_%s_report.txt', fault);
if exist(rep, 'file'), delete(rep); end
diary(rep);
cleanup = onCleanup(@() diary('off'));
fprintf('build_conveyor_step4(%s)  %s  MATLAB %s\n\n', fault, datestr(now), version);

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
B.seal  = addb(mdl, 'Rotational Friction',             'DrumSeals',    [500 300 560 340]);
B.mref5 = addb(mdl, 'Mechanical Rotational Reference', 'MRef5',        [500 360 540 400]);
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
% thermal network
B.tsen_b = addb(mdl, 'Ideal Torque Sensor',             'BrgTorque',    [600 220 660 280]);
B.wsrc   = addb(mdl, 'Ideal Torque Source',             'WearTorque',   [600 400 660 460]);
B.sps_w  = addb(mdl, 'Simulink-PS Converter',           'Twear_in',     [520 430 550 450]);
B.mref4  = addb(mdl, 'Mechanical Rotational Reference', 'MRef4',        [700 470 740 510]);
B.amb    = addb(mdl, 'Controlled Temperature Source',   'Ambient',      [300 700 360 760]);
B.thref1 = addb(mdl, 'Thermal Reference',               'ThRef1',       [300 820 340 860]);
B.thref2 = addb(mdl, 'Thermal Reference',               'ThRef2',       [700 820 740 860]);
B.thref3 = addb(mdl, 'Thermal Reference',               'ThRef3',       [500 820 540 860]);
B.thref4 = addb(mdl, 'Thermal Reference',               'ThRef4',       [900 820 940 860]);
B.thref5 = addb(mdl, 'Thermal Reference',               'ThRef5',       [100 820 140 860]);
B.cv_mot = addb(mdl, 'Convective Heat Transfer',        'MotorCooling', [420 600 480 640]);
B.cv_brg = addb(mdl, 'Convective Heat Transfer',        'BrgCooling',   [620 600 680 640]);
B.m_brg  = addb(mdl, 'Thermal Mass',                    'BrgMass',      [760 560 810 610]);
B.q_brg  = addb(mdl, 'Controlled Heat Flow Rate Source','BrgHeat',      [760 680 820 740]);
B.ts_mot = addb(mdl, 'Temperature Sensor',              'MotTempSens',  [420 480 480 540]);
B.ts_brg = addb(mdl, 'Temperature Sensor',              'BrgTempSens',  [860 480 920 540]);
B.ts_amb = addb(mdl, 'Temperature Sensor',              'AmbTempSens',  [100 600 160 660]);
B.sps_a  = addb(mdl, 'Simulink-PS Converter',           'Tamb_in',      [200 760 230 780]);
B.sps_q  = addb(mdl, 'Simulink-PS Converter',           'Qbrg_in',      [700 760 730 780]);
B.pss_tm = addb(mdl, 'PS-Simulink Converter',           'Tmot_out',     [520 480 550 500]);
B.pss_tb = addb(mdl, 'PS-Simulink Converter',           'Tbrg_out',     [960 480 990 500]);
B.pss_ta = addb(mdl, 'PS-Simulink Converter',           'Tamb_out',     [200 600 230 620]);
B.pss_tq = addb(mdl, 'PS-Simulink Converter',           'Tbrg_trq',     [700 240 730 260]);
% motor: switch on the built-in thermal model (block variant with thermal port H)
[motorThermal, MP] = motor_thermal(B.motor);
if ~motorThermal
    % fallback: copper loss I^2*R computed outside, into a separate winding thermal mass
    B.m_mot = addb(mdl, 'Thermal Mass',                     'MotMass',  [300 480 350 530]);
    B.q_mot = addb(mdl, 'Controlled Heat Flow Rate Source', 'MotHeat',  [300 560 360 620]);
    B.sps_qm = addb(mdl, 'Simulink-PS Converter',           'Qmot_in',  [220 580 250 600]);
end

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
logs = {'speed_u', 'current_A', 'voltage_V', 'setpoint_u', 'belt_u', 'load_N', 'grip_mode', 'Tmot_C', 'Tbrg_C', 'Tamb_C', 'Pbrg_W', 'severity', 'running'};
for q = 1:numel(logs)
    add_block('simulink/Sinks/To Workspace', [mdl '/log_' logs{q}], 'Position', [1500 40+q*40 1560 60+q*40], ...
        'VariableName', logs{q}, 'SaveFormat', 'Timeseries');
end

% ambient: 22 degC + slow random drift, fed in kelvin
add_block('simulink/Sources/Band-Limited White Noise', [mdl '/NoiseAmb'], 'Position', [-100 700 -60 730], ...
    'Cov', num2str(2 * P.tau_amb * P.sig_amb^2), 'Ts', '1', 'seed', num2str(P.seed + 2000));
add_block('simulink/Continuous/Transfer Fcn', [mdl '/OUamb'], 'Position', [-40 695 40 735], ...
    'Numerator', '[1]', 'Denominator', sprintf('[%g 1]', P.tau_amb));
add_block('simulink/Sources/Constant', [mdl '/AmbMeanK'], 'Position', [-40 640 20 660], 'Value', num2str(P.T_amb0 + 273.15));
add_block('simulink/Math Operations/Sum', [mdl '/AmbSum'], 'Position', [80 690 100 730], 'Inputs', '++');
% bearing friction loss = |friction torque x drum speed|
add_block('simulink/Math Operations/Gain', [mdl '/w_drum'], 'Position', [640 300 680 320], 'Gain', num2str(1 / P.N));
add_block('simulink/Math Operations/Product', [mdl '/BrgLoss'], 'Position', [740 300 770 340]);
add_block('simulink/Math Operations/Abs', [mdl '/BrgLossAbs'], 'Position', [790 305 820 335]);
add_block('simulink/Math Operations/Gain', [mdl '/QSign'], 'Position', [830 305 860 335], 'Gain', num2str(P.qSign));
% 0.5 s lag on the heat signal: a real Simulink state that breaks the loop
% (speed, torque -> loss -> heat input). Negligible next to the 120 s thermal time constant.
add_block('simulink/Continuous/Transfer Fcn', [mdl '/QLag'], 'Position', [870 305 930 335], ...
    'Numerator', '[1]', 'Denominator', '[0.5 1]');
% temperatures K -> degC
for q = {'Tmot', 'Tbrg', 'Tamb'}
    add_block('simulink/Math Operations/Bias', [mdl '/' q{1} '_C'], 'Position', [1000 400 1040 420], 'Bias', '-273.15');
end
if ~motorThermal
    add_block('simulink/Math Operations/Product', [mdl '/CuLoss'], 'Position', [140 560 170 600], 'Inputs', '***');
    add_block('simulink/Sources/Constant', [mdl '/Ra_const'], 'Position', [80 600 110 620], 'Value', num2str(P.Ra));
    add_block('simulink/Continuous/Transfer Fcn', [mdl '/QmLag'], 'Position', [180 560 240 600], ...
        'Numerator', '[1]', 'Denominator', '[0.5 1]');
end

% ---- fault severity: s = 0.8 * ((t - t_inj)/T)^2 (or linear for jam), capped at 1
add_block('simulink/Sources/Clock', [mdl '/Clock'], 'Position', [-420 900 -390 930]);
add_block('simulink/Math Operations/Bias', [mdl '/SinceInj'], 'Position', [-360 900 -320 930], 'Bias', num2str(-P.t_inj));
add_block('simulink/Discontinuities/Saturation', [mdl '/NotBefore'], 'Position', [-300 900 -260 930], ...
    'UpperLimit', 'inf', 'LowerLimit', '0');
add_block('simulink/Math Operations/Gain', [mdl '/PerT'], 'Position', [-240 900 -200 930], 'Gain', num2str(1 / P.T));
if P.quad.(fault)
    add_block('simulink/Math Operations/Math Function', [mdl '/Shape'], 'Position', [-180 900 -140 930], 'Operator', 'square');
else
    add_block('simulink/Math Operations/Gain', [mdl '/Shape'], 'Position', [-180 900 -140 930], 'Gain', '1');
end
add_block('simulink/Math Operations/Gain', [mdl '/To08'], 'Position', [-120 900 -80 930], 'Gain', '0.8');
add_block('simulink/Discontinuities/Saturation', [mdl '/Sev'], 'Position', [-60 900 -20 930], ...
    'UpperLimit', '1', 'LowerLimit', '0');
if strcmp(fault, 'none')
    set_param([mdl '/To08'], 'Gain', '0');
end
% trip: running = (s < 0.8); the motor voltage is multiplied by it
add_block('simulink/Sources/Constant', [mdl '/TripLevel'], 'Position', [0 960 30 980], 'Value', '0.8');
add_block('simulink/Logic and Bit Operations/Relational Operator', [mdl '/Running'], 'Position', [40 900 70 940], 'Operator', '<');
add_block('simulink/Signal Attributes/Data Type Conversion', [mdl '/RunD'], 'Position', [90 905 130 935], 'OutDataTypeStr', 'double');
add_block('simulink/Math Operations/Product', [mdl '/VoltCut'], 'Position', [20 -120 50 -80]);
% jam and overload: extra load force
add_block('simulink/Math Operations/Gain', [mdl '/JamForce'], 'Position', [160 1000 200 1020], ...
    'Gain', num2str(strcmp(fault, 'jam') * P.F_jam));
add_block('simulink/Math Operations/Gain', [mdl '/OvForce'], 'Position', [160 1040 200 1060], ...
    'Gain', num2str(strcmp(fault, 'overload') * P.F_ov));
% slip: grip factor (1 - s), never below 1 %%
add_block('simulink/Math Operations/Gain', [mdl '/SlipS'], 'Position', [160 1080 200 1100], ...
    'Gain', num2str(double(strcmp(fault, 'slip'))));
add_block('simulink/Math Operations/Gain', [mdl '/Neg'], 'Position', [220 1120 260 1140], 'Gain', '-1');
add_block('simulink/Math Operations/Bias', [mdl '/GripFac'], 'Position', [280 1120 320 1140], 'Bias', '1');
add_block('simulink/Discontinuities/Saturation', [mdl '/GripFacSat'], 'Position', [340 1120 380 1140], ...
    'UpperLimit', '1', 'LowerLimit', '0.01');
for c = {'Kin', 'Pos', 'Neg'}
    add_block('simulink/Math Operations/Product', [mdl '/Grip' c{1}], 'Position', [400 1100 430 1140]);
end
% wear: extra bearing torque, opposing drum rotation, through a lag (breaks the loop)
add_block('simulink/Math Operations/Gain', [mdl '/WearT'], 'Position', [160 1160 200 1180], ...
    'Gain', num2str(strcmp(fault, 'wear') * P.T_wear));
add_block('simulink/Math Operations/Gain', [mdl '/WdScale'], 'Position', [160 1200 200 1220], 'Gain', '2');
add_block('simulink/Math Operations/Trigonometric Function', [mdl '/WdDir'], 'Position', [220 1200 260 1220], 'Operator', 'tanh');
add_block('simulink/Math Operations/Product', [mdl '/WearDir'], 'Position', [280 1160 310 1200]);
add_block('simulink/Math Operations/Gain', [mdl '/WearSign'], 'Position', [330 1165 360 1195], 'Gain', num2str(P.wearSign));
add_block('simulink/Continuous/Transfer Fcn', [mdl '/WearLag'], 'Position', [380 1165 440 1195], ...
    'Numerator', '[1]', 'Denominator', '[0.05 1]');
set_param([mdl '/LoadSum'], 'Inputs', '+++++');

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
setp(mdl, 'DrumSeals', {'brkwy_trq'}, num2str(1.1 * P.Tc_seal));
setp(mdl, 'DrumSeals', {'Col_trq'}, num2str(P.Tc_seal));
setp(mdl, 'DrumSeals', {'visc_coef'}, num2str(P.b_seal));
setp(mdl, 'Wheel', {'R'}, num2str(P.r));        setp(mdl, 'Wheel', {'R_unit'}, 'm');
setp(mdl, 'Belt', {'mass'}, num2str(P.mb));     setp(mdl, 'Belt', {'mass_unit'}, 'kg');
setp(mdl, 'Rollers', {'brkwy_frc'}, num2str(1.1 * P.Fc_rol));
setp(mdl, 'Rollers', {'brkwy_vel'}, '0.01');
setp(mdl, 'Rollers', {'Col_frc'}, num2str(P.Fc_rol));
setp(mdl, 'Rollers', {'visc_coef'}, num2str(P.b_rol));
setp(mdl, 'Grip', {'initial_state_locked'}, 'sdl.enum.initial_lock.locked');
for c = {'Tk_in', 'Tp_in', 'Tn_in'}
    setp(mdl, c{1}, {'Unit'}, 'N*m');
    % the grip now changes during a run (slip fault), so the clutch needs its derivative:
    % let the converter filter it (10 ms) and compute it
    setp(mdl, c{1}, {'FilteringAndDerivatives'}, 'filter');
    setp(mdl, c{1}, {'InputFilterTimeConstant'}, '0.01');
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
for c = {'Tmot_out', 'Tbrg_out', 'Tamb_out'}, setp(mdl, c{1}, {'Unit'}, 'K'); end
setp(mdl, 'Tamb_in', {'Unit'}, 'K');
setp(mdl, 'Tamb_in', {'FilteringAndDerivatives'}, 'filter');
setp(mdl, 'Tamb_in', {'InputFilterTimeConstant'}, '1');
setp(mdl, 'Qbrg_in', {'Unit'}, 'W');
setp(mdl, 'Qbrg_in', {'FilteringAndDerivatives'}, 'filter');
setp(mdl, 'Qbrg_in', {'InputFilterTimeConstant'}, '0.1');
setp(mdl, 'Tbrg_trq', {'Unit'}, 'N*m');
setp(mdl, 'Twear_in', {'Unit'}, 'N*m');
setp(mdl, 'Twear_in', {'FilteringAndDerivatives'}, 'filter');
setp(mdl, 'Twear_in', {'InputFilterTimeConstant'}, '0.01');
setp(mdl, 'MotorCooling', {'area'}, '1');  setp(mdl, 'MotorCooling', {'area_unit'}, 'm^2');
setp(mdl, 'MotorCooling', {'heat_tr_coeff'}, num2str(P.hA_mot)); setp(mdl, 'MotorCooling', {'heat_tr_coeff_unit'}, 'W/(K*m^2)');
setp(mdl, 'BrgCooling', {'area'}, '1');    setp(mdl, 'BrgCooling', {'area_unit'}, 'm^2');
setp(mdl, 'BrgCooling', {'heat_tr_coeff'}, num2str(P.hA_brg)); setp(mdl, 'BrgCooling', {'heat_tr_coeff_unit'}, 'W/(K*m^2)');
setp(mdl, 'BrgMass', {'mass'}, '1');       setp(mdl, 'BrgMass', {'mass_unit'}, 'kg');
setp(mdl, 'BrgMass', {'sp_heat'}, num2str(P.C_brg)); setp(mdl, 'BrgMass', {'sp_heat_unit'}, 'J/(K*kg)');
setp(mdl, 'BrgMass', {'T_specify'}, 'on'); setp(mdl, 'BrgMass', {'T'}, num2str(P.T_amb0 + 273.15)); setp(mdl, 'BrgMass', {'T_unit'}, 'K');
if motorThermal
    setp(mdl, 'Motor', {'thermal_mass'}, num2str(P.C_mot));  setp(mdl, 'Motor', {'thermal_mass_unit'}, 'J/K');
    setp(mdl, 'Motor', {'initial_temperature'}, num2str(P.T_amb0)); setp(mdl, 'Motor', {'initial_temperature_unit'}, 'degC');
else
    setp(mdl, 'MotMass', {'mass'}, '1');   setp(mdl, 'MotMass', {'sp_heat'}, num2str(P.C_mot));
    setp(mdl, 'MotMass', {'T_specify'}, 'on'); setp(mdl, 'MotMass', {'T'}, num2str(P.T_amb0 + 273.15)); setp(mdl, 'MotMass', {'T_unit'}, 'K');
    setp(mdl, 'Qmot_in', {'Unit'}, 'W');
    setp(mdl, 'Qmot_in', {'FilteringAndDerivatives'}, 'filter');
    setp(mdl, 'Qmot_in', {'InputFilterTimeConstant'}, '0.1');
end

%% ---------------- physical connections ----------------
fprintf('\n---- port geometry (for checking) ----\n');
dumpports(B.motor); dumpports(B.tsen_b); dumpports(B.amb); dumpports(B.q_brg); dumpports(B.ts_mot);
% resolve all port handles before drawing lines
T.sup_p = pp(B.cvs, 'top', 1);      T.sup_V = pp(B.cvs, 'bottom', 1);   T.sup_n = pp(B.cvs, 'bottom', 2);
T.cs_p  = pp(B.cs, 'left', 1);      T.cs_I  = pp(B.cs, 'right', 1);     T.cs_n  = pp(B.cs, 'right', 2);
T.mot_p = MP.p;  T.mot_R = MP.R;  T.mot_n = MP.n;  T.mot_C = MP.C;
T.gear_B = pp(B.gear, 'left', 1);   T.gear_F = pp(B.gear, 'right', 1);
T.brg_R = pp(B.brg, 'left', 1);     T.brg_C = pp(B.brg, 'right', 1);
T.seal_R = pp(B.seal, 'left', 1);   T.seal_C = pp(B.seal, 'right', 1);  T.mref5 = pp(B.mref5, 'only', 1);
% torque sensor: left R; right C (top), T (signal, bottom)
T.tqs_R = pp(B.tsen_b, 'left', 1);  T.tqs_C = pp(B.tsen_b, 'right', 1); T.tqs_T = pp(B.tsen_b, 'right', 2);
T.ws_Rw = pp(B.wsrc, 'top', 1);     T.ws_Sw = pp(B.wsrc, 'bottom', 1);  T.ws_Cw = pp(B.wsrc, 'bottom', 2);
T.sps_w = pp(B.sps_w, 'only', 1);   T.mref4 = pp(B.mref4, 'only', 1);
% thermal blocks
T.amb_B = pp(B.amb, 'top', 1);      T.amb_S = pp(B.amb, 'bottom', 1);   T.amb_A = pp(B.amb, 'bottom', 2);
T.q_B   = pp(B.q_brg, 'top', 1);    T.q_S   = pp(B.q_brg, 'bottom', 1); T.q_A   = pp(B.q_brg, 'bottom', 2);
T.cvm_A = pp(B.cv_mot, 'left', 1);  T.cvm_B = pp(B.cv_mot, 'right', 1);
T.cvb_A = pp(B.cv_brg, 'left', 1);  T.cvb_B = pp(B.cv_brg, 'right', 1);
T.mbrg  = pp(B.m_brg, 'only', 1);
T.tsm_A = pp(B.ts_mot, 'left', 1);  T.tsm_B = pp(B.ts_mot, 'right', 1); T.tsm_T = pp(B.ts_mot, 'right', 2);
T.tsb_A = pp(B.ts_brg, 'left', 1);  T.tsb_B = pp(B.ts_brg, 'right', 1); T.tsb_T = pp(B.ts_brg, 'right', 2);
T.tsa_A = pp(B.ts_amb, 'left', 1);  T.tsa_B = pp(B.ts_amb, 'right', 1); T.tsa_T = pp(B.ts_amb, 'right', 2);
T.thr1 = pp(B.thref1, 'only', 1);   T.thr2 = pp(B.thref2, 'only', 1);   T.thr3 = pp(B.thref3, 'only', 1);
T.thr4 = pp(B.thref4, 'only', 1);   T.thr5 = pp(B.thref5, 'only', 1);
T.sps_a = pp(B.sps_a, 'only', 1);   T.sps_q = pp(B.sps_q, 'only', 1);
T.pss_tm = pp(B.pss_tm, 'only', 1); T.pss_tb = pp(B.pss_tb, 'only', 1);
T.pss_ta = pp(B.pss_ta, 'only', 1); T.pss_tq = pp(B.pss_tq, 'only', 1);
if motorThermal
    T.mot_H = MP.H;
else
    T.mot_H = pp(B.m_mot, 'only', 1);
    T.qm_B = pp(B.q_mot, 'top', 1); T.qm_S = pp(B.q_mot, 'bottom', 1); T.qm_A = pp(B.q_mot, 'bottom', 2);
    T.sps_qm = pp(B.sps_qm, 'only', 1);
end
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
conn(mdl, T.gear_F, T.drum);  conn(mdl, T.gear_F, T.tqs_R); conn(mdl, T.tqs_C, T.brg_R);
conn(mdl, T.brg_C, T.mref2);  conn(mdl, T.tqs_T, T.pss_tq);
conn(mdl, T.gear_F, T.seal_R); conn(mdl, T.seal_C, T.mref5);
% wear torque acts in the bearing branch, behind the torque sensor, so its heat is counted
conn(mdl, T.ws_Rw, T.tqs_C);  conn(mdl, T.ws_Cw, T.mref4);  conn(mdl, T.sps_w, T.ws_Sw);
% thermal: ambient node = Ambient source B (A to reference)
conn(mdl, T.amb_A, T.thr1);   conn(mdl, T.sps_a, T.amb_S);
conn(mdl, T.mot_H, T.cvm_A);  conn(mdl, T.cvm_B, T.amb_B);
conn(mdl, T.mbrg, T.cvb_A);   conn(mdl, T.cvb_B, T.amb_B);
conn(mdl, T.q_B, T.mbrg);     conn(mdl, T.q_A, T.thr2);     conn(mdl, T.sps_q, T.q_S);
conn(mdl, T.tsm_A, T.mot_H);  conn(mdl, T.tsm_B, T.thr3);   conn(mdl, T.tsm_T, T.pss_tm);
conn(mdl, T.tsb_A, T.mbrg);   conn(mdl, T.tsb_B, T.thr4);   conn(mdl, T.tsb_T, T.pss_tb);
conn(mdl, T.tsa_A, T.amb_B);  conn(mdl, T.tsa_B, T.thr5);   conn(mdl, T.tsa_T, T.pss_ta);
if ~motorThermal
    conn(mdl, T.qm_B, T.mot_H); conn(mdl, T.qm_A, T.thr2); conn(mdl, T.sps_qm, T.qm_S);
end
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
sl(mdl, 'PI/1', 'Driver/1');        sl(mdl, 'Driver/1', 'VoltCut/1'); sl(mdl, 'RunD/1', 'VoltCut/2');
sl(mdl, 'VoltCut/1', 'V_in/1');     sl(mdl, 'VoltCut/1', 'log_voltage_V/1');
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
sl(mdl, 'Tkin/1', 'GripKin/1');     sl(mdl, 'GripFacSat/1', 'GripKin/2'); sl(mdl, 'GripKin/1', 'Tk_in/1');
sl(mdl, 'Tpos/1', 'GripPos/1');     sl(mdl, 'GripFacSat/1', 'GripPos/2'); sl(mdl, 'GripPos/1', 'Tp_in/1');
sl(mdl, 'Tneg/1', 'GripNeg/1');     sl(mdl, 'GripFacSat/1', 'GripNeg/2'); sl(mdl, 'GripNeg/1', 'Tn_in/1');
% fault chain
sl(mdl, 'Clock/1', 'SinceInj/1');   sl(mdl, 'SinceInj/1', 'NotBefore/1'); sl(mdl, 'NotBefore/1', 'PerT/1');
sl(mdl, 'PerT/1', 'Shape/1');       sl(mdl, 'Shape/1', 'To08/1');      sl(mdl, 'To08/1', 'Sev/1');
sl(mdl, 'Sev/1', 'Running/1');      sl(mdl, 'TripLevel/1', 'Running/2'); sl(mdl, 'Running/1', 'RunD/1');
sl(mdl, 'Sev/1', 'log_severity/1'); sl(mdl, 'RunD/1', 'log_running/1');
sl(mdl, 'Sev/1', 'JamForce/1');     sl(mdl, 'JamForce/1', 'LoadSum/4');
sl(mdl, 'Sev/1', 'OvForce/1');      sl(mdl, 'OvForce/1', 'LoadSum/5');
sl(mdl, 'Sev/1', 'SlipS/1');        sl(mdl, 'SlipS/1', 'Neg/1');       sl(mdl, 'Neg/1', 'GripFac/1');
sl(mdl, 'GripFac/1', 'GripFacSat/1');
sl(mdl, 'Sev/1', 'WearT/1');        sl(mdl, 'WearT/1', 'WearDir/1');
sl(mdl, 'w_drum/1', 'WdScale/1');   sl(mdl, 'WdScale/1', 'WdDir/1');   sl(mdl, 'WdDir/1', 'WearDir/2');
sl(mdl, 'WearDir/1', 'WearSign/1'); sl(mdl, 'WearSign/1', 'WearLag/1'); sl(mdl, 'WearLag/1', 'Twear_in/1');
sl(mdl, 'NoiseAmb/1', 'OUamb/1');   sl(mdl, 'AmbMeanK/1', 'AmbSum/1'); sl(mdl, 'OUamb/1', 'AmbSum/2');
sl(mdl, 'AmbSum/1', 'Tamb_in/1');
sl(mdl, 'W_out/1', 'w_drum/1');     sl(mdl, 'w_drum/1', 'BrgLoss/1'); sl(mdl, 'Tbrg_trq/1', 'BrgLoss/2');
sl(mdl, 'BrgLoss/1', 'BrgLossAbs/1'); sl(mdl, 'BrgLossAbs/1', 'QSign/1'); sl(mdl, 'QSign/1', 'QLag/1'); sl(mdl, 'QLag/1', 'Qbrg_in/1'); sl(mdl, 'BrgLossAbs/1', 'log_Pbrg_W/1');
sl(mdl, 'Tmot_out/1', 'Tmot_C/1');  sl(mdl, 'Tmot_C/1', 'log_Tmot_C/1');
sl(mdl, 'Tbrg_out/1', 'Tbrg_C/1');  sl(mdl, 'Tbrg_C/1', 'log_Tbrg_C/1');
sl(mdl, 'Tamb_out/1', 'Tamb_C/1');  sl(mdl, 'Tamb_C/1', 'log_Tamb_C/1');
if ~motorThermal
    sl(mdl, 'I_out/1', 'CuLoss/1'); sl(mdl, 'I_out/1', 'CuLoss/2'); sl(mdl, 'Ra_const/1', 'CuLoss/3');
    sl(mdl, 'CuLoss/1', 'QmLag/1'); sl(mdl, 'QmLag/1', 'Qmot_in/1');
end

set_param(mdl, 'StopTime', num2str(P.Tstop), 'Solver', 'ode23t', 'RelTol', '1e-4', 'MaxStep', '0.05', ...
    'ZeroCrossAlgorithm', 'Adaptive', 'IgnoredZcDiagnostic', 'none');
save_system(mdl);
fprintf('\nModel %s.slx built and saved.\n', mdl);

%% ---------------- simulate and report ----------------
out = sim(mdl);
u  = out.get('speed_u');  i_ = out.get('current_A');  v = out.get('voltage_V');
ub = out.get('belt_u');   fl = out.get('load_N');     gm = out.get('grip_mode');
t  = u.Time;  ss = t >= 30 & t < P.t_inj;          % healthy checks: before the fault only
ubi = rs(ub, t(ss));            % all signals on the motor-speed time grid
ii  = rs(i_, t(ss));
fli = rs(fl, t(ss));
vi  = rs(v, t(ss));
ratio = abs(ubi) ./ max(abs(u.Data(ss)), 1e-6);
fprintf('\n---- healthy part (30 s to injection) ----\n');
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

tm = out.get('Tmot_C'); tb = out.get('Tbrg_C'); ta = out.get('Tamb_C'); pb = out.get('Pbrg_W');
te = t >= P.t_inj - 60 & t < P.t_inj;              % last 60 s before the fault
Ta = mean(rs(ta, t(te)));
fprintf('\n---- temperatures (last 60 s before injection) ----\n');
fprintf('motor thermal model   %s\n', ternary(motorThermal, 'built-in DC Motor thermal variant (R rises with T)', 'FALLBACK: external I^2*R, constant R'));
fprintf('ambient               %6.2f degC\n', Ta);
fprintf('motor winding         %6.2f degC   rise %5.2f K   (PLC model at speed 60: +30.0 K)\n', mean(rs(tm, t(te))), mean(rs(tm, t(te))) - Ta);
fprintf('drum bearing          %6.2f degC   rise %5.2f K   (PLC model at speed 60: +10.5 K)\n', mean(rs(tb, t(te))), mean(rs(tb, t(te))) - Ta);
fprintf('bearing loss          %6.1f W\n', mean(rs(pb, t(te))));
if mean(rs(tb, t(te))) < Ta, fprintf('WARNING: bearing below ambient: set P.qSign = -1 and run again.\n'); end
if Ta < 0 || Ta > 60, fprintf('WARNING: ambient temperature implausible: ambient source sign reversed.\n'); end
% time to 63 %% of the final rise (first-order time constant check)
for q = {{'motor', tm, 60}, {'bearing', tb, 120}}
    ts_ = q{1}{2}; tq = (0:0.5:P.t_inj)'; y = rs(ts_, tq) - P.T_amb0; yend = y(end);
    k63 = find(y >= 0.632 * yend, 1);
    if ~isempty(k63), fprintf('%-8s 63%% of rise after %5.0f s from start   (PLC time constant %d s, plus ~2 s ramp)\n', q{1}{1}, tq(k63), q{1}{3}); end
end

sv = out.get('severity'); rn = out.get('running');
tq = (0:0.25:P.Tstop)';
svq = rs(sv, tq); rnq = rs(rn, tq);
kt = find(rnq < 0.5, 1);
fprintf('\n---- fault: %s ----\n', fault);
fprintf('injected at %g s, nominal time to trip %g s\n', P.t_inj, P.T);
if isempty(kt)
    fprintf('NO TRIP within the run\n');
    t_trip = P.Tstop;
else
    t_trip = tq(kt);
    fprintf('trip at %.2f s  ->  %.2f s after injection\n', t_trip, t_trip - P.t_inj);
end
pre  = tq >= P.t_inj - 30 & tq < P.t_inj;
late = tq >= max(P.t_inj, t_trip - max(1, 0.1 * P.T)) & tq < t_trip;
sig = {'current (A)', i_; 'motor temp (degC)', tm; 'bearing temp (degC)', tb; 'voltage (V)', v; 'bearing loss (W)', pb};
fprintf('%-22s %10s %14s %10s\n', 'signal', 'before', 'just before trip', 'change');
for q = 1:size(sig, 1)
    y = rs(sig{q, 2}, tq);
    fprintf('%-22s %10.2f %14.2f %+10.2f\n', sig{q, 1}, mean(y(pre)), mean(y(late)), mean(y(late)) - mean(y(pre)));
end
uq = rs(u, tq); ubq = abs(rs(ub, tq));
rq = ubq ./ max(abs(uq), 1e-6);
fprintf('%-22s %10.4f %14.4f %+10.4f\n', 'belt ratio', mean(rq(pre)), mean(rq(late)), mean(rq(late)) - mean(rq(pre)));
if strcmp(fault, 'wear')
    % sign check on the bearing branch itself: an opposing wear torque raises the friction
    % loss. (The current is no good for this check: the wear share, ~0.35 A at the trip,
    % is smaller than the slow load wander, std ~0.23 A.)
    yp = rs(pb, tq);
    if mean(yp(late)) < mean(yp(pre)), fprintf('WARNING: wear lowered the bearing loss: set P.wearSign = -1 and run again.\n'); end
end

figure('Name', sprintf('conveyor_step4 %s', fault));
subplot(4,1,1); plot(tq, svq, tq, rnq, '--'); ylabel('severity / running'); grid on; xline(P.t_inj, ':');
subplot(4,1,2); plot(i_.Time, i_.Data); ylabel('current (A)'); grid on; xline(P.t_inj, ':');
subplot(4,1,3); plot(tm.Time, tm.Data, tb.Time, tb.Data); ylabel('temp (degC)'); legend('motor', 'bearing', 'Location', 'west'); grid on; xline(P.t_inj, ':');
subplot(4,1,4); plot(tq, rq); ylabel('belt ratio'); xlabel('time (s)'); grid on; xline(P.t_inj, ':');
ylim([0 1.1]);
fprintf('\nDone. Send the ---- fault block and a screenshot of the figure.\n');
end

%% ======================= helpers =======================
function out = ternary(c, a, b)
if c, out = a; else, out = b; end
end

function [ok, MP] = motor_thermal(h)
% Remember where the four standard ports sit, switch the DC Motor to its thermal variant,
% then find each port again by its position; the new port is the thermal port H.
[edges, xy, hs] = portedges(h);
bp = get_param(h, 'Position');
top = find(strcmp(edges, 'top'));    [~, o] = sort(xy(top, 1));    top = top(o);
bot = find(strcmp(edges, 'bottom')); [~, o] = sort(xy(bot, 1));    bot = bot(o);
rel = @(k) xy(k, :) - bp(1:2);
ref = struct('p', rel(top(1)), 'R', rel(top(2)), 'n', rel(bot(1)), 'C', rel(bot(2)));
MP = struct('p', hs(top(1)), 'R', hs(top(2)), 'n', hs(bot(1)), 'C', hs(bot(2)), 'H', []);
ok = false;
for route = {'SourceFile', 'ComponentPath', 'ComponentVariants'}
    try
        set_param(h, route{1}, 'ee.electromech.brushed.dc_motor_thermal');
    catch err
        fprintf('motor thermal via %-18s: %s\n', route{1}, strtok(err.message, newline));
        continue;
    end
    ph = get_param(h, 'PortHandles');
    n = numel(ph.LConn) + numel(ph.RConn);
    fprintf('motor thermal via %-18s: accepted, %d physical ports\n', route{1}, n);
    if n == 5, ok = true; break; end
end
if ~ok
    fprintf('motor thermal variant NOT available: using the fallback (external I^2*R heat).\n');
    return;
end
[~, xy2, hs2] = portedges(h);
bp2 = get_param(h, 'Position');
r2 = xy2 - bp2(1:2);
used = false(numel(hs2), 1);
for f = {'p', 'R', 'n', 'C'}
    d = sum(abs(r2 - ref.(f{1})), 2);
    [dm, k] = min(d + used * 1e9);
    if dm > 6, error('motor port %s moved after the variant switch; send the port geometry', f{1}); end
    MP.(f{1}) = hs2(k); used(k) = true;
end
MP.H = hs2(~used);
end

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
