proj_root = fileparts(fileparts(mfilename('fullpath')));
run(fullfile(proj_root, 'setup_paths.m'));

cfg = SimulationConfig();
world = ScenarioDefinitions('handdrawn_village_advanced', cfg);

x = 144.24; y = 2.42 + 0.145; theta = 0.21;
L = cfg.vehicle_length; W = cfg.vehicle_width; margin = cfg.safety_margin;

half_L = L / 2 + margin;
half_W = W / 2 + margin;
corners_local = [ half_L,  half_W;
                  half_L, -half_W;
                 -half_L, -half_W;
                 -half_L,  half_W ];

rot = [cos(theta), -sin(theta); sin(theta), cos(theta)];
corners_world = (rot * corners_local')' + [x, y];

fprintf('Vehicle center with off-tracking compensation: x=%.2f, y=%.2f, th=%.2f rad\n', x, y, theta);
for c = 1:4
    px = corners_world(c, 1);
    py = corners_world(c, 2);
    [y_min, y_max] = world.road_geometry.getBounds(px);
    in_c = (py >= y_min && py <= y_max);
    fprintf('Corner %d: px=%6.2f, py=%5.2f, bounds=[%5.2f, %5.2f], in_bounds=%d (margin=%.3fm)\n', ...
        c, px, py, y_min, y_max, in_c, min(py - y_min, y_max - py));
end
