function grid = buildDobTuningGrid(config)
% buildDobTuningGrid - Cartesian DOB search grid from the experiment contract.
[cutoff, forceLimit, rateLimit] = ndgrid(config.dobCutoffHzGrid, ...
    config.dobForceLimitGrid, config.dobRateLimitGrid);
grid = table(cutoff(:), forceLimit(:), rateLimit(:), ...
    'VariableNames', {'cutoff_hz', 'leg_force_limit_n', 'rate_limit_nps'});
grid.id = string(compose('dob_f%g_l%g_r%g', grid.cutoff_hz, ...
    grid.leg_force_limit_n, grid.rate_limit_nps));
grid = movevars(grid, 'id', 'Before', 1);
end
