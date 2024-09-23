function cost = BallStageCost(x, u, dt, mode_type, CostParams)
    cost = 0;
    cost = cost + PositionCost(x, CostParams);
    % if mode_type == 1
    %     cost = cost * 0.001;
    % end
    cost = cost + VelocityCost(x, CostParams);
    cost = cost + InputCost(u, CostParams);
    cost = cost*dt;
end

function cost = PositionCost(x, CostParams)
    cost = dot(CostParams.position_weight.*(x(1:2) - CostParams.position_target), (x(1:2) - CostParams.position_target));
end

function cost = VelocityCost(x, CostParams)
    cost = dot(CostParams.velocity_weight.*(x(3:4) - CostParams.velocity_target), (x(3:4) - CostParams.velocity_target));
end

function cost = InputCost(u, CostParams)
    cost = dot(CostParams.u_weight.*(u - CostParams.u_target), (u - CostParams.u_target));
end
