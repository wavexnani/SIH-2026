proj_root = fileparts(fileparts(mfilename('fullpath')));
run(fullfile(proj_root, 'setup_paths.m'));

json_path = fullfile(proj_root, 'web_viewer', 'data', 'handdrawn_village_advanced.json');
data = jsondecode(fileread(json_path));

cfg = SimulationConfig();
world = ScenarioDefinitions('handdrawn_village_advanced', cfg);

bad_steps = [];
for k = 1:length(data.timesteps)
    if iscell(data.timesteps), ts = data.timesteps{k}; else, ts = data.timesteps(k); end
    in_b = world.road_geometry.isFootprintInBounds(ts.ego.x, ts.ego.y, ts.ego.theta, ...
                                                   cfg.vehicle_length, cfg.vehicle_width, cfg.safety_margin);
    if ~in_b
        [y_min, y_max] = world.road_geometry.getBounds(ts.ego.x);
        [y_c, th_r, kap] = world.road_geometry.getCenterline(ts.ego.x);
        bad_steps = [bad_steps; k, ts.time, ts.ego.x, ts.ego.y, ts.ego.theta, ts.ego.v, y_c, y_min, y_max];
    end
end

fprintf('Total out-of-bounds steps: %d / %d\n', size(bad_steps, 1), length(data.timesteps));
if ~isempty(bad_steps)
    fprintf('Step  Time   ego.x   ego.y  ego.th   ego.v    y_c    y_min   y_max\n');
    for i = 1:min(20, size(bad_steps, 1))
        fprintf('%4d  %5.2f  %6.2f  %6.2f  %6.2f  %5.2f  %6.2f  %6.2f  %6.2f\n', bad_steps(i, :));
    end
    if size(bad_steps, 1) > 20
        fprintf('...\n');
        for i = (size(bad_steps, 1)-5):size(bad_steps, 1)
            fprintf('%4d  %5.2f  %6.2f  %6.2f  %6.2f  %5.2f  %6.2f  %6.2f  %6.2f\n', bad_steps(i, :));
        end
    end
end
