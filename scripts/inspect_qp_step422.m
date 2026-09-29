proj_root = fileparts(fileparts(mfilename('fullpath')));
run(fullfile(proj_root, 'setup_paths.m'));

json_path = fullfile(proj_root, 'web_viewer', 'data', 'handdrawn_village_advanced.json');
data = jsondecode(fileread(json_path));

cfg = SimulationConfig();
world = ScenarioDefinitions('handdrawn_village_advanced', cfg);
ctrl5 = Stage5CoordinationController(cfg);

ts = data.timesteps(422);
world.ego.x = ts.ego.x;
world.ego.y = ts.ego.y;
world.ego.theta = ts.ego.theta;
world.ego.v = ts.ego.v;
world.ego.delta = ts.ego.delta;
world.ego.a = ts.ego.a;

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

% Inspect CACRC internal QP solution
disp('CACRC Planner info fields:');
disp(fieldnames(info));
