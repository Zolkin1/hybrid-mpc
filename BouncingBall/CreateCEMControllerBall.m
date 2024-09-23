function  controller = CreateCEMControllerBall(T, x0, BallParams, GuardParams, CostParams)
%CREATECEMCONTROLLERBALL Summary of this function goes here
%   

% Create Cost Function
costfcn = @(xsample) CostFunction(xsample, x0, T, BallParams, GuardParams, CostParams);

% Call CEMOptimization
ntime_steps = T*15;

cem_settings.nsamples = 50; %75; %120;
cem_settings.xsize = 2*ntime_steps;
cem_settings.var = 250*eye(cem_settings.xsize); % 0.125
cem_settings.mean = -20*ones(cem_settings.xsize, 1);
cem_settings.nelite = 1; % 3
cem_settings.max_iters = 15; %15;

[cemean, cevar, yopt] = CrossEntropyOptimization(cem_settings, costfcn);

yopt

% TODO: Remove
%cemean = zeros(length(cemean), 1);

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
cost = SimCost(sol, xsample, CostParams);
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

function cost = SimCost(sol, xsample, CostParams)
    cost = 0;
    t = linspace(0, sol(end).x(end), sol(end).x(end)*30); % 30 cost evaluations per second
    t_idx = 1;
    dt = t(2) - t(1);

    nodes = length(xsample)/2;

    ux = xsample(1:nodes);
    uy = xsample(nodes + 1:nodes*2);

    tsample = linspace(0, sol(end).x(end), nodes);

    for j = 1:length(sol)
        while t_idx <= length(t) && t(t_idx) <= sol(j).x(end)
            u = zeros(2, 1);
            u(1) = interp1(tsample, ux, t(t_idx));
            u(2) = interp1(tsample, uy, t(t_idx));

            cost = cost + BallStageCost(deval(sol(j), t(t_idx)), u, dt, 2, CostParams);
            t_idx = t_idx + 1;
        end
    end

    if t_idx ~= length(t) + 1
        error("bad t indexing!");
    end
end

