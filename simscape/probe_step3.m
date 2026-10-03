% probe_step3.m  (BeltTwin option A, step 3)
% 1) Shows how the DC Motor's thermal model can be switched on in this release
%    (block variants / thermal-port options), and the motor's ports with it switched on.
% 2) Places the thermal blocks needed for step 3 into a test model "probe3_model" and prints
%    their port geometry and main parameters.

libs = {'fl_lib', 'ee_lib', 'sdl_lib', 'nesl_utility', 'simulink'};
for i = 1:numel(libs), load_system(libs{i}); end
mdl = 'probe3_model';
if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist([mdl '.slx'], 'file'), delete([mdl '.slx']); end
new_system(mdl);
fprintf('\nMATLAB %s\n', version);

%% 1) DC motor thermal options
h = add_block('ee_lib/Electromechanical/Brushed Motors/DC Motor', [mdl '/Motor'], 'Position', [40 40 140 140]);
fprintf('\n==== DC Motor: thermal options\n');
props = {'ComponentVariantNames', 'ComponentVariants', 'ComponentPath', 'SourceFile', 'BlockChoice'};
for p = props
    try
        v = get_param(h, p{1});
        if iscell(v), v = strjoin(v, ' | '); end
        if ~ischar(v), v = mat2str(v); end
        fprintf('    %-24s %s\n', p{1}, v);
    catch
        fprintf('    %-24s (not available)\n', p{1});
    end
end
dp = fieldnames(get_param(h, 'DialogParameters'));
hits = dp(~cellfun(@isempty, regexpi(dp, 'therm|temp|port|model', 'once')));
fprintf('    thermal-related parameters: %s\n', strjoin(hits', ', '));
for k = 1:numel(hits)
    try fprintf('        %-26s = %s\n', hits{k}, char(get_param(h, hits{k}))); catch, end
end
% try switching thermal on by the usual routes, report what worked
routes = {
    {'thermal_port', 'simscape.enum.thermaleffects.model'}
    {'thermal_effects', 'simscape.enum.thermaleffects.model'}
    {'thermal_port', 'model'} };
for r = 1:numel(routes)
    try
        set_param(h, routes{r}{1}, routes{r}{2});
        fprintf('    set %s = %s : OK\n', routes{r}{1}, routes{r}{2});
    catch err
        fprintf('    set %s = %s : %s\n', routes{r}{1}, routes{r}{2}, strtok(err.message, newline));
    end
end
try
    names = get_param(h, 'ComponentVariantNames');
    if ischar(names), names = {names}; end
    for q = 1:numel(names)
        if ~isempty(regexpi(names{q}, 'therm', 'once'))
            set_param(h, 'ComponentVariants', names{q});
            fprintf('    set ComponentVariants = %s : OK\n', names{q});
        end
    end
catch err
    fprintf('    ComponentVariants switch: %s\n', strtok(err.message, newline));
end
ph = get_param(h, 'PortHandles');
fprintf('    motor ports now: LConn %d, RConn %d\n', numel(ph.LConn), numel(ph.RConn));
dumpports(h);

%% 2) thermal blocks
names = {'Temperature Sensor', 'Thermal Reference', 'Controlled Temperature Source', ...
         'Ideal Torque Sensor', 'Thermal Mass', 'Convective Heat Transfer', ...
         'Controlled Heat Flow Rate Source'};
norm = @(s) lower(strtrim(regexprep(s, '\s+', ' ')));
skip = '(_unit|_conf|_specify|_priority|_nominal_value|_nominal_unit|_nominal_specify)$';
for n = 1:numel(names)
    path = '';
    for i = 1:4
        try
            c = find_system(libs{i}, 'SearchDepth', 3, 'LookUnderMasks', 'all', 'FollowLinks', 'on', ...
                'MatchFilter', @Simulink.match.allVariants, 'Type', 'block');
        catch
            c = find_system(libs{i}, 'SearchDepth', 3, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'Type', 'block');
        end
        for k = 1:numel(c)
            if strcmp(norm(get_param(c{k}, 'Name')), norm(names{n}))
                if isempty(path) || numel(c{k}) < numel(path), path = c{k}; end
            end
        end
        if ~isempty(path), break; end
    end
    fprintf('\n==== %s\n', names{n});
    if isempty(path), fprintf('    NOT FOUND\n'); continue; end
    fprintf('    path: %s\n', strrep(path, newline, ' '));
    x = 240 + mod(n - 1, 4) * 260; y = 40 + floor((n - 1) / 4) * 220;
    hb = add_block(path, sprintf('%s/B%02d', mdl, n), 'Position', [x y x+100 y+100]);
    set_param(hb, 'Name', sprintf('%02d %s', n, names{n}));
    dumpports(hb);
    dpb = fieldnames(get_param(hb, 'DialogParameters'));
    fprintf('    parameters:\n');
    for j = 1:numel(dpb)
        if ~isempty(regexp(dpb{j}, skip, 'once')), continue; end
        try v = get_param(hb, dpb{j}); catch, v = '?'; end
        if ~ischar(v), v = '<non-text>'; end
        u = ''; try u = get_param(hb, [dpb{j} '_unit']); catch, end
        fprintf('        %-30s = %s %s\n', dpb{j}, strrep(v, newline, ' '), u);
    end
end

save_system(mdl);
open_system(mdl);
fprintf('\nSaved and opened %s.slx: take a screenshot with the port labels readable.\n', mdl);

function dumpports(h)
ph = get_param(h, 'PortHandles');
hs = [ph.LConn(:); ph.RConn(:)];
tags = [arrayfun(@(q) sprintf('LConn%d', q), 1:numel(ph.LConn), 'UniformOutput', false), ...
        arrayfun(@(q) sprintf('RConn%d', q), 1:numel(ph.RConn), 'UniformOutput', false)];
bp = get_param(h, 'Position');
en = {'left', 'top', 'right', 'bottom'};
fprintf('    ports:\n');
for q = 1:numel(hs)
    pos = get_param(hs(q), 'Position');
    d = [abs(pos(1) - bp(1)), abs(pos(2) - bp(2)), abs(pos(1) - bp(3)), abs(pos(2) - bp(4))];
    [~, e] = min(d);
    fprintf('        %-7s %-7s at (%d, %d)\n', tags{q}, en{e}, pos(1), pos(2));
end
end
