function ok=c3SolverGate(success,status,equality,inequality,bounds,finite,withinBudget,physical)
ok=logical(success) && strcmp(status,'Solve_Succeeded') && finite && withinBudget && physical && ...
    all(isfinite([equality,inequality,bounds])) && all([equality,inequality,bounds]<=1e-6);
end
