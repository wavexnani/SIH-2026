proj_root = fileparts(fileparts(mfilename('fullpath')));
run(fullfile(proj_root, 'setup_paths.m'));

json_path = fullfile(proj_root, 'web_viewer', 'data', 'handdrawn_village_advanced.json');
data = jsondecode(fileread(json_path));

cfg = SimulationConfig();
rg = RoadGeometry('unstructured', 'road_length', 210.0, ...
                  'road_width', cfg.road_width, 'y_center', cfg.road_center_y, ...
                  'curve_amp', 1.50, 'curve_lambda', 80.0, 'grade_slope', 0.01);
rg.blind_bend_active = true;
rg.blind_bend_amp = 16.0;
rg.blind_bend_x_start = 135.0;
rg.blind_bend_length = 46.0;
rg.boundary_noise_amp = 0.08;
rg.setBridgeCanal(92.0, 106.0, 5.2);

N = length(data.timesteps);
fprintf('Total timesteps: %d\n', N);

bad_steps = [];
for k = 1:N
    if iscell(data.timesteps)
        ts = data.timesteps{k};
    else
        ts = data.timesteps(k);
    end
    ego = ts.ego;
    in_b = rg.isFootprintInBounds(ego.x, ego.y, ego.theta, cfg.vehicle_length, cfg.vehicle_width, cfg.safety_margin);
    if ~in_b
        [y_min, y_max] = rg.getBounds(ego.x);
        [y_c, th_r, kap] = rg.getCenterline(ego.x);
        bad_steps = [bad_steps; k, ts.time, ego.x, ego.y, ego.theta, ego.v, y_c, y_min, y_max];
    end
end

fprintf('Total out-of-bounds steps: %d / %d\n', size(bad_steps, 1), N);
if ~isempty(bad_steps)
    fprintf('Step  Time   ego.x   ego.y  ego.th   ego.v    y_c    y_min   y_max\n');
    for i = 1:min(15, size(bad_steps, 1))
        fprintf('%4d  %5.2f  %6.2f  %6.2f  %6.2f  %5.2f  %6.2f  %6.2f  %6.2f\n', bad_steps(i, :));
    end
    if size(bad_steps, 1) > 15
        fprintf('...\n');
        for i = max(16, size(bad_steps, 1)-5):size(bad_steps, 1)
            fprintf('%4d  %5.2f  %6.2f  %6.2f  %6.2f  %5.2f  %6.2f  %6.2f  %6.2f\n', bad_steps(i, :));
        end
    end
end
