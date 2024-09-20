function pos = SwingTrajectoryCasadi(t, node, total_nodes, T0, Tf, x0, xf, apex, height_s, height_f)
%SWINGTRAJECTORYCASADI Summary of this function goes here
%   
apex_time = 0.2*(Tf - T0) + T0;
apex_node = ceil(0.2*total_nodes);
swing_time = (Tf - T0);

pos = [0*x0;0*x0];

% X position is strict interpolation
pos(1) = ((t - T0)/(Tf - T0))*(xf - x0) + x0; %interp1([T0, Tf], [x0, xf], t);

%p = pchip([T0 - 1, T0, apex_time, Tf, Tf + 1], [height_s, height_s, apex, height_f, height_f]);

%pos(2) = ppval(p, t);

if node < apex_node
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

