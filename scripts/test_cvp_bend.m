proj_root = fileparts(fileparts(mfilename('fullpath')));
run(fullfile(proj_root, 'setup_paths.m'));

cfg = SimulationConfig();
world = ScenarioDefinitions('handdrawn_village_advanced', cfg);

N_path = 600;
road_len = world.road_length;
ref_path = zeros(N_path, 5);
ref_path(:, 1) = linspace(0, road_len, N_path)';
y_offset_lane = world.ego.y - world.road_geometry.y_center_base;
for r = 1:N_path
    [y_c_r, th_r, kap_r] = world.road_geometry.getCenterline(ref_path(r, 1));
    ref_path(r, 2) = y_c_r + y_offset_lane;
    ref_path(r, 3) = th_r;
    ref_path(r, 4) = kap_r;
end
ref_path(:, 5) = 4.5;

[~, idx_149] = min(abs(ref_path(:, 1) - 148.90));
disp('ref_path around 148.90:');
disp(ref_path(max(1, idx_149-2):min(N_path, idx_149+5), :));

% Now test CurvatureVelocityPlanner
cvp = CurvatureVelocityPlanner(cfg);
aug = cvp.planVelocityProfile(ref_path, 4.5);
disp('aug_path with curvature velocity planner around 148.90:');
disp(aug(max(1, idx_149-2):min(N_path, idx_149+5), :));
