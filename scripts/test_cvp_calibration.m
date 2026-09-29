proj_root = fileparts(fileparts(mfilename('fullpath')));
run(fullfile(proj_root, 'setup_paths.m'));

cfg = SimulationConfig();
world = ScenarioDefinitions('handdrawn_village_advanced', cfg);

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

% Windowed curvature envelope (20m lookahead window)
k_raw = abs(ref_path(:, 4));
k_env = k_raw;
ds = road_len / N_path;
w_pts = round(20.0 / ds); % ~57 points for 20m
for r = 1:N_path
    idx_lo = max(1, r - w_pts);
    idx_hi = min(N_path, r + w_pts);
    k_env(r) = max(k_raw(idx_lo:idx_hi));
end

ref_path_env = ref_path;
ref_path_env(:, 4) = k_env;

cvp = CurvatureVelocityPlanner(cfg);
cvp.a_lat_max = 0.40;
cvp.v_min = 3.20;
aug = cvp.planVelocityProfile(ref_path_env, 4.5);

disp('x, y, theta, kappa_env, v_profile with Curvature Envelope:');
for r = 1:N_path
    if ref_path(r, 1) >= 130 && ref_path(r, 1) <= 185 && mod(r, 5) == 0
        fprintf('x=%6.2f, y=%5.2f, th=%5.2f, kap_env=%7.4f, v_plan=%5.2f\n', ...
            aug(r, 1), aug(r, 2), aug(r, 3), aug(r, 4), aug(r, 5));
    end
end
