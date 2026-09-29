proj_root = fileparts(fileparts(mfilename('fullpath')));
run(fullfile(proj_root, 'setup_paths.m'));

json_path = fullfile(proj_root, 'web_viewer', 'data', 'handdrawn_village_advanced.json');
data = jsondecode(fileread(json_path));

cfg = SimulationConfig();
world = ScenarioDefinitions('handdrawn_village_advanced', cfg);
ctrl5 = Stage5CoordinationController(cfg);

% Set world.ego to step 422 state
ts = data.timesteps(422);
world.ego.x = ts.ego.x;
world.ego.y = ts.ego.y;
world.ego.theta = ts.ego.theta;
world.ego.v = ts.ego.v;
world.ego.delta = ts.ego.delta;
world.ego.a = ts.ego.a;

% No active agents nearby at x=148.90
for a_i = 1:world.n_agents
    world.agents(a_i).x = -100;
    world.agents(a_i).y = -100;
    world.agents(a_i).v = 0;
end

N_path = 600;
road_len = world.road_length;
ref_path = zeros(N_path, 5);
ref_path(:, 1) = linspace(0, road_len, N_path)';
y_offset_lane = -1.20;
for r = 1:N_path
    [y_c_r, th_r, kap_r] = world.road_geometry.getCenterline(ref_path(r, 1));
    ref_path(r, 2) = y_c_r + y_offset_lane;
    ref_path(r, 3) = th_r;
    ref_path(r, 4) = kap_r;
end
ref_path(:, 5) = 4.5;

[u_cmd, pred_states, status, info] = ctrl5.step(world, ref_path, 4.5);
fprintf('Ego: x=%.2f, y=%.2f, v=%.2f, th=%.2f\n', world.ego.x, world.ego.y, world.ego.v, world.ego.theta);
fprintf('Command: delta=%.4f (%.1f deg), a=%.4f\n', u_cmd(1), rad2deg(u_cmd(1)), u_cmd(2));
fprintf('Macro Intent: %s, target_v_coord=%.2f\n', info.macro_intent, info.target_v_coord);
fprintf('CACRC status: %d, is_soft: %d, filter_active: %d, filter_reason: %s\n', ...
    status, info.is_soft, info.filter_active, info.filter_reason);
if isfield(info, 'pred_ego_traj') && ~isempty(info.pred_ego_traj)
    disp('Predicted Ego Trajectory (first 5 steps) [x, y, theta, v]:');
    disp(info.pred_ego_traj(1:min(5, size(info.pred_ego_traj, 1)), :));
end
% Print nearest reference trajectory points
ref_xy = ref_path(:, 1:2);
dists = (ref_xy(:, 1) - world.ego.x).^2 + (ref_xy(:, 2) - world.ego.y).^2;
[~, nearest_idx] = min(dists);
disp('Reference Trajectory [x, y, theta, v] from nearest_idx:');
disp(ref_path(nearest_idx:min(nearest_idx+4, size(ref_path, 1)), :));

