proj_root = fileparts(fileparts(mfilename('fullpath')));
run(fullfile(proj_root, 'setup_paths.m'));

rg = RoadGeometry('unstructured', 'road_length', 210.0, 'curve_amp', 1.50);
rg.blind_bend_active = true;
rg.blind_bend_amp = 16.0;
rg.blind_bend_x_start = 135.0;
rg.blind_bend_length = 46.0;
x_vec = linspace(135, 181, 100);
kaps = zeros(size(x_vec));
ths = zeros(size(x_vec));
for i = 1:length(x_vec)
    [~, ths(i), kaps(i)] = rg.getCenterline(x_vec(i));
end
fprintf('Max |kappa|: %.4f (min radius: %.2f m)\n', max(abs(kaps)), 1/max(abs(kaps)));
fprintf('Max heading th: %.2f rad (%.1f deg)\n', max(ths), rad2deg(max(ths)));
