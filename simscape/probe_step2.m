% probe_step2.m  (BeltTwin option A, step 2)
% Places the blocks needed for the belt (friction clutch, wheel and axle, belt mass,
% translational friction, force source, translational motion sensor) into a test model
% "probe2_model", prints each block's port geometry and its main parameter names, and
% opens the model so the port labels can be read off the icons.

names = { ...
    'Fundamental Friction Clutch', 'Disk Friction Clutch', 'Belt Pulley', ...
    'Wheel and Axle', 'Mass', 'Translational Friction', 'Ideal Force Source', ...
    'Ideal Translational Motion Sensor', 'Mechanical Translational Reference', ...
    'Band-Limited White Noise'};

libs = {'fl_lib', 'ee_lib', 'sdl_lib', 'nesl_utility', 'simulink'};
for i = 1:numel(libs), load_system(libs{i}); end

mdl = 'probe2_model';
if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist([mdl '.slx'], 'file'), delete([mdl '.slx']); end
new_system(mdl);

norm = @(s) lower(strtrim(regexprep(s, '\s+', ' ')));
skip = '(_unit|_conf|_specify|_priority|_nominal_value|_nominal_unit|_nominal_specify)$';
fprintf('\nMATLAB %s\n', version);

for n = 1:numel(names)
    path = '';
    for i = 1:numel(libs)
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
    x = 40 + mod(n - 1, 4) * 300; y = 40 + floor((n - 1) / 4) * 220;
    h = add_block(path, sprintf('%s/B%02d', mdl, n), 'Position', [x y x+100 y+100]);
    set_param(h, 'Name', sprintf('%02d %s', n, names{n}));

    % port geometry
    ph = get_param(h, 'PortHandles');
    hs = [ph.LConn(:); ph.RConn(:)];
    tags = [arrayfun(@(q) sprintf('LConn%d', q), 1:numel(ph.LConn), 'UniformOutput', false), ...
            arrayfun(@(q) sprintf('RConn%d', q), 1:numel(ph.RConn), 'UniformOutput', false)];
    bp = get_param(h, 'Position');
    edgeNames = {'left', 'top', 'right', 'bottom'};
    fprintf('    ports (Simulink in %d, out %d):\n', numel(ph.Inport), numel(ph.Outport));
    for q = 1:numel(hs)
        pos = get_param(hs(q), 'Position');
        d = [abs(pos(1) - bp(1)), abs(pos(2) - bp(2)), abs(pos(1) - bp(3)), abs(pos(2) - bp(4))];
        [~, e] = min(d);
        fprintf('        %-7s %-7s at (%d, %d)\n', tags{q}, edgeNames{e}, pos(1), pos(2));
    end

    % main parameters (no unit/configuration noise)
    dp = fieldnames(get_param(h, 'DialogParameters'));
    fprintf('    parameters:\n');
    for j = 1:numel(dp)
        if ~isempty(regexp(dp{j}, skip, 'once')), continue; end
        try v = get_param(h, dp{j}); catch, v = '?'; end
        if ~ischar(v), v = '<non-text>'; end
        u = '';
        try u = get_param(h, [dp{j} '_unit']); catch, end
        fprintf('        %-30s = %s %s\n', dp{j}, strrep(v, newline, ' '), u);
    end
end

save_system(mdl);
open_system(mdl);
fprintf('\nSaved and opened %s.slx: take a screenshot with the port labels readable.\n', mdl);
