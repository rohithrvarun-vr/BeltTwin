% port_probe.m  (BeltTwin option A)
% Places every Simscape block the conveyor model needs into a test model "port_probe",
% prints each block's library path and port counts, and opens the model so the port
% labels can be read off the block icons. Nothing here is part of the final model.
% Blocks are found by name at run time (whitespace and line breaks in library names ignored).

names = { ...
    'DC Motor', 'Controlled Voltage Source', 'Current Sensor', 'Electrical Reference', ...
    'Simple Gear', 'Inertia', 'Rotational Friction', 'Ideal Torque Source', ...
    'Ideal Rotational Motion Sensor', 'Mechanical Rotational Reference', ...
    'Wheel and Axle', 'Mass', 'Translational Friction', 'Ideal Force Source', ...
    'Ideal Translational Motion Sensor', 'Mechanical Translational Reference', ...
    'Thermal Mass', 'Convective Heat Transfer', 'Temperature Source', ...
    'Controlled Heat Flow Rate Source', 'Ideal Temperature Sensor', ...
    'Solver Configuration', 'Simulink-PS Converter', 'PS-Simulink Converter'};

libs = {'fl_lib', 'ee_lib', 'sdl_lib', 'nesl_utility'};
for i = 1:numel(libs)
    load_system(libs{i});
end

mdl = 'port_probe';
if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist([mdl '.slx'], 'file'), delete([mdl '.slx']); end
new_system(mdl);

norm = @(s) lower(strtrim(regexprep(s, '\s+', ' ')));
fprintf('\nMATLAB %s\n\n', version);
fprintf('%-36s %-6s %-6s %-6s %-6s  %s\n', 'BLOCK', 'LConn', 'RConn', 'In', 'Out', 'LIBRARY PATH');

col = 0; row = 0;
for n = 1:numel(names)
    path = '';
    for i = 1:numel(libs)
        try
            c = find_system(libs{i}, 'SearchDepth', 3, 'LookUnderMasks', 'all', 'FollowLinks', 'on', ...
                'MatchFilter', @Simulink.match.allVariants, 'Type', 'block');
        catch
            c = find_system(libs{i}, 'SearchDepth', 3, 'LookUnderMasks', 'all', 'FollowLinks', 'on', ...
                'Type', 'block');
        end
        for k = 1:numel(c)
            if strcmp(norm(get_param(c{k}, 'Name')), norm(names{n}))
                if isempty(path) || numel(c{k}) < numel(path)
                    path = c{k};
                end
            end
        end
        if ~isempty(path), break; end
    end
    if isempty(path)
        fprintf('%-36s NOT FOUND\n', names{n});
        continue;
    end
    x = 40 + col * 260; y = 40 + row * 200;
    dst = sprintf('%s/B%02d', mdl, n);
    h = add_block(path, dst, 'Position', [x y x+90 y+90]);
    set_param(h, 'Name', sprintf('%02d %s', n, names{n}));
    ph = get_param(h, 'PortHandles');
    fprintf('%-36s %-6d %-6d %-6d %-6d  %s\n', names{n}, numel(ph.LConn), numel(ph.RConn), ...
        numel(ph.Inport), numel(ph.Outport), strrep(path, newline, ' '));
    col = col + 1;
    if col == 6, col = 0; row = row + 1; end
end

save_system(mdl);
open_system(mdl);
fprintf('\nSaved and opened %s.slx: take screenshots with the port labels readable.\n', mdl);
