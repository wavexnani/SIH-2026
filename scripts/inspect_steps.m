proj_root = fileparts(fileparts(mfilename('fullpath')));
run(fullfile(proj_root, 'setup_paths.m'));

json_path = fullfile(proj_root, 'web_viewer', 'data', 'handdrawn_village_advanced.json');
data = jsondecode(fileread(json_path));

for k = 418:432
    if iscell(data.timesteps)
        ts = data.timesteps{k};
    else
        ts = data.timesteps(k);
    end
    fprintf('Step %3d: filter_active=%d, filter_reason=%-20s, mpc.u=[%5.2f, %5.2f], mpc.status=%d, mpc.is_soft=%d\n', ...
        k, ts.safety.filter_active, ts.safety.filter_reason, ts.mpc.u_cmd(1), ts.mpc.u_cmd(2), ts.mpc.status, ts.mpc.is_soft);
end
