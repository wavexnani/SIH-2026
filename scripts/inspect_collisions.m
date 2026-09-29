proj_root = fileparts(fileparts(mfilename('fullpath')));
run(fullfile(proj_root, 'setup_paths.m'));

json_path = fullfile(proj_root, 'web_viewer', 'data', 'handdrawn_village_advanced.json');
data = jsondecode(fileread(json_path));

cfg = SimulationConfig();
world = ScenarioDefinitions('handdrawn_village_advanced', cfg);

coll_steps = [];
for k = 1:length(data.timesteps)
    if iscell(data.timesteps), ts = data.timesteps{k}; else, ts = data.timesteps(k); end
    % check collision
    world.ego.x = ts.ego.x; world.ego.y = ts.ego.y; world.ego.theta = ts.ego.theta;
    world.ego.v = ts.ego.v;
    clr = world.getMinClearance(cfg);
    if clr <= 0
        coll_steps = [coll_steps; k, ts.time, ts.ego.x, ts.ego.y, clr];
    end
end
fprintf('Total collision steps: %d\n', size(coll_steps, 1));
if ~isempty(coll_steps)
    disp('First 10 collision steps [k, time, x, y, clearance]:');
    disp(coll_steps(1:min(10, end), :));
    disp('Last 5 collision steps:');
    disp(coll_steps(max(1, end-4):end, :));
end
