function pos = SwingTrajectory(t, T0, Tf, x0, xf, apex, height_s, height_f)
%SWINGTRAJECTORY Characterizes a swing trajectory
%   Detailed explanation goes here

apex_time = 0.2*(Tf - T0) + T0;
swing_time = (Tf - T0);

pos = zeros(2,1);

% X position is strict interpolation
pos(1) = interp1([T0, Tf], [x0, xf], t);

if t < apex_time
    % First spline
    pos(2) = height_s - ...
    (apex_time - T0)^-2 * 3 *(height_s - apex) * (t - T0)^2 + ...
    (apex_time - T0)^-3 * 2 * (height_s - apex) * (t - T0)^3;
else
    % Second spline
    pos(2) = apex - ...
    (Tf - apex_time)^-2 * 3 *(-height_f + apex) * (t - apex_time)^2 + ...
    (Tf - apex_time)^-3 * 2 * (-height_f + apex) * (t - apex_time)^3;
end

end

