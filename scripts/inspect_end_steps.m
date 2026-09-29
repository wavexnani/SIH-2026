proj_root = fileparts(fileparts(mfilename('fullpath')));
run(fullfile(proj_root, 'setup_paths.m'));

json_path = fullfile(proj_root, 'web_viewer', 'data', 'handdrawn_village_advanced.json');
data = jsondecode(fileread(json_path));

for k = 460:10:550
    if iscell(data.timesteps), ts = data.timesteps{k}; else, ts = data.timesteps(k); end
    fprintf('Step %3d: t=%5.2f, ego.x=%6.2f, ego.y=%5.2f, ego.v=%5.2f, th=%5.2f, intent=%s\n', ...
        k, ts.time, ts.ego.x, ts.ego.y, ts.ego.v, ts.ego.theta, ts.decision.macro_intent);
end

% Check bounds failures
bad = [];
for k = 1:length(data.timesteps)
    if iscell(data.timesteps), ts = data.timesteps{k}; else, ts = data.timesteps(k); end
    if ~ts.inside_bounds % if inside_bounds exists or check footprint
        bad = [bad; k, ts.ego.x, ts.ego.y, ts.ego.theta, ts.ego.v];
    end
end
fprintf('\nTotal bad steps recorded in history: %d\n', size(bad, 1));
if ~isempty(bad)
    fprintf('First 10 bad steps:\n');
    disp(bad(1:min(10, end), :));
    fprintf('Last 10 bad steps:\n');
    disp(bad(max(1, end-9):end, :));
end
