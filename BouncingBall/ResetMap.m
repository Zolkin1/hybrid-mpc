function x0 = ResetMap(sol, BallParams, GuardParams)
x0 = sol.y(:, end);
if ~isempty(sol.ie)
    if sol.ie(1) == 1 || sol.ie(1) == 2
        x0 = WallReset(sol.ie(1), sol.y(:, end), BallParams, GuardParams);
    end
end
end

