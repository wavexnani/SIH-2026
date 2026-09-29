proj_root = fileparts(fileparts(mfilename('fullpath')));
run(fullfile(proj_root, 'setup_paths.m'));

json_path = fullfile(proj_root, 'web_viewer', 'data', 'handdrawn_village_advanced.json');
data = jsondecode(fileread(json_path));

ts = data.timesteps(480);
fprintf('Step 480 (t=%.2fs): ego.x=%.2f, ego.y=%.2f, ego.v=%.2f, ego.th=%.2f\n', ...
    ts.time, ts.ego.x, ts.ego.y, ts.ego.v, ts.ego.theta);
fprintf('Macro Intent: %s, target_v=%.2f\n', ts.decision.macro_intent, ts.decision.target_v);
fprintf('Safety filter active: %d, reason: %s\n', ts.safety.filter_active, ts.safety.filter_reason);

disp('Nearby Observed Agents:');
for i = 1:length(ts.observation.agents)
    ag = ts.observation.agents(i);
    dx = ag.x - ts.ego.x;
    dy = ag.y - ts.ego.y;
    d = hypot(dx, dy);
    if d < 40.0
        fprintf('  Agent %d (%s): x=%.2f, y=%.2f, vx=%.2f, vy=%.2f, dx=%.2f, dy=%.2f, dist=%.2f\n', ...
            ag.id, ag.type, ag.x, ag.y, ag.vx, ag.vy, dx, dy, d);
    end
end
