function  controller = CreateCEMControllerBall(T, x0, BallParams, GuardParams, CostParams)
%CREATECEMCONTROLLERBALL Summary of this function goes here
%   

% Create Cost Function
costfcn = @(xsample) CostFunction(xsample, x0, T, BallParams, GuardParams, CostParams);

% Call CEMOptimization
ntime_steps = T*15;

cem_settings.nsamples = 120;
cem_settings.xsize = 2*ntime_steps;
cem_settings.var = 250*eye(cem_settings.xsize); % 0.125
cem_settings.mean = 0*ones(cem_settings.xsize, 1);
cem_settings.nelite = 3;
cem_settings.max_iters = 15;

[cemean, cevar, yopt] = CrossEntropyOptimization(cem_settings, costfcn);

yopt

% Create the controller
controller.Compute = @(t, x) SampleController(t, cemean, T);

end

function cost = CostFunction(xsample, x0, T, BallParams, GuardParams, CostParams)

% Create the controller from the samples
controller.Compute = @(t, x) SampleController(t, xsample, T);

% Simulate through the end of the time
tsim = 0;
sol = [];
while tsim < T
    sol = [sol, BallSimulation(tsim, T, x0, controller, BallParams, GuardParams)];
    x0 = ResetMap(sol(end), BallParams, GuardParams);
    tsim = sol(end).x(end);
end

% Evaluate the cost
cost = SimCost(sol, CostParams);
cost = cost + InputCost(xsample, T, CostParams);
end

function u = SampleController(t, xsample, T)
    nodes = length(xsample)/2;

    ux = xsample(1:nodes);
    uy = xsample(nodes + 1:nodes*2);

    tsample = linspace(0, T, nodes);
    u = zeros(2, 1);
    u(1) = interp1(tsample, ux, t);
    u(2) = interp1(tsample, uy, t);
end

function cost = SimCost(sol, CostParams)
    cost = 0;
    for j = 1:length(sol)
        for i = 1:length(sol(j).x) - 1
            dt = sol(j).x(i + 1) - sol(j).x(i);
            cost = cost + dt*StageCost(sol(j).y(:, i), CostParams);
        end
    end
end

function cost = StageCost(x, CostParams)
    cost = 0;
    cost = cost + PositionCost(x, CostParams);
    cost = cost + VelocityCost(x, CostParams);
end

function cost = PositionCost(x, CostParams)
    cost = dot(CostParams.position_weight.*(x(1:2) - CostParams.position_target), (x(1:2) - CostParams.position_target));
end

function cost = VelocityCost(x, CostParams)
    cost = dot(CostParams.velocity_weight.*(x(3:4) - CostParams.velocity_target), (x(3:4) - CostParams.velocity_target));
end

function cost = InputCost(xsample, T, CostParams)
    cost = 0;
    nodes = length(xsample)/2;
    ux = xsample(1:nodes);
    uy = xsample(nodes + 1:nodes*2);
    dt = T/nodes;
    for i = 1:nodes
        cost = cost + dt*dot(CostParams.u_weight.*([ux(i); uy(i)] - CostParams.u_target), ([ux(i); uy(i)] - CostParams.u_target));
    end
end


