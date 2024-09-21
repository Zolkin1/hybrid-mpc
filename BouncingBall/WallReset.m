function xplus = WallReset(guard_idx, x, BallParams, GuardParams)
    xplus = x;
    xplus(GuardParams.Direction{guard_idx}(2) + 2) = -BallParams.gamma*x(GuardParams.Direction{guard_idx}(2) + 2);
end
