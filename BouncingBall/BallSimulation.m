function sol = BallSimulation(t0, tf, x0, controller, BallParams, GuardParams)
    options = odeset('Events', @(t,x) GuardConditions(t, x, BallParams, GuardParams), 'Refine', 8);
    sol = ode45(@(t, x) BallDynamics(t, x, controller, BallParams, GuardParams), [t0, tf], x0, options);
end

function dxdt = BallDynamics(t, x, controller, BallParams, GuardParams)
    dxdt = zeros(4, 1);
    dxdt(1:2) = x(3:4);
    if norm(x(1:2) - GuardParams.G{3}(1:2)) < GuardParams.G{3}(3)
        dxdt(3:4) = BallParams.drag*norm(x(3:4))^2*x(3:4) + ...
            min(max(controller.Compute(t, x), -BallParams.ForceLim), BallParams.ForceLim)/BallParams.M;
    else
        dxdt(3:4) = BallParams.drag*norm(x(3:4))^2*x(3:4) + BallParams.g +...
            max(min(controller.Compute(t, x), [0; 0]), -BallParams.ForceLim)/BallParams.M;
    end
end

function [value, isterminal, direction] = GuardConditions(t, x, BallParams, GuardParams)
    value = ones(GuardParams.num, 1);
    isterminal = ones(GuardParams.num, 1);
    direction = ones(GuardParams.num, 1);

    for i = 1:GuardParams.num
        if i == 1 || i == 2
            if GuardParams.Direction{i}(2) == 1
                if x(2) >= GuardParams.Direction{i}(3) && x(2) <= GuardParams.Direction{i}(4)
                    value(i) = x(1) - BallParams.r - GuardParams.Value(i);
                else
                    value(i) = 1;
                end
            else
                if x(1) >= GuardParams.Direction{i}(3) && x(1) <= GuardParams.Direction{i}(4)
                    value(i) = x(2) - BallParams.r - GuardParams.Value(i);
                else
                    value(i) = 1;
                end
            end
        else
            direction(i) = -1;
            value(i) = norm(x(1:2) - GuardParams.G{3}(1:2)) - GuardParams.G{3}(3);
            value(i);
        end

        direction(i) = GuardParams.Direction{i}(1);
    end
end
