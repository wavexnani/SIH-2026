proj_root = fileparts(fileparts(mfilename('fullpath')));
run(fullfile(proj_root, 'setup_paths.m'));

json_path = fullfile(proj_root, 'web_viewer', 'data', 'handdrawn_village_advanced.json');
data = jsondecode(fileread(json_path));

for k = 420:426
    if iscell(data.timesteps), ts = data.timesteps{k}; else, ts = data.timesteps(k); end
    fprintf('Step %3d: selected=%s, ok_L=%d, ok_R=%d, J_L=%.1f, J_R=%.1f\n', ...
        k, ts.topology.selected, ts.topology.ok_geom_left, ts.topology.ok_geom_right, ts.topology.J_left, ts.topology.J_right);
end
