function xplus = WallReset(guard_idx, x, BallParams, GuardParams)
    posplus = x(1:2);

    idx = GuardParams.Direction{guard_idx}(2) + 2;
    
    if idx == 3
        xplus = [posplus; -BallParams.gamma(2)*x(idx); x(4)];
    else
        xplus = [posplus; x(3); -BallParams.gamma(1)*x(idx)];
    end
end
