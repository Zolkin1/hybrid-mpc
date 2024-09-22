clc;
clear;
close all;

% TODO: Consider adding gravity back in the box

%%
BallParams.M = 1;
BallParams.r = 0.1;
BallParams.g = [0; -9.81];
BallParams.gamma = [1; 1]; % 0.8
BallParams.drag = -0.02;

BallParams.ForceLim = [100; 100];

pos0 = [1, 1];


GuardParams.num = 3;
GuardParams.G{1} = [0, 0; 8, 0]; % Wall on the x axis
GuardParams.G{2} = [0, 0; 0, 8]; % Wall on the y axis
GuardParams.G{3} = [3, 3, 1]; % x, y, radius

GuardParams.Color(1) = "k";
GuardParams.Color(2) = "k";
GuardParams.Color(3) = "b";

GuardParams.Direction{1} = [-1, 2, 0, 8]; % inc.dec value, x/y constraint, start value, end value
GuardParams.Direction{2} = [-1, 1, 0, 8];
GuardParams.Direction{3} = [0, 1, 2, 4];

GuardParams.Value(1) = 0;
GuardParams.Value(2) = 0;
GuardParams.Value(3) = 2;

%% Cost Parameters
CostParams.position_target = [3; 3];
CostParams.position_weight = [50; 50];
CostParams.velocity_target = [0; 0];
CostParams.velocity_weight = 0.001*[1; 1];
CostParams.u_target = [0; 0];
CostParams.u_weight = 0.001*[1; 1];

costfcn = @(x, u, dt) BallStageCost(x, u, dt, CostParams);

%%
t0 = 0;
tf = 3; %5;

controller.Compute = @(t, x) ZeroController();

mpc_dt = 0.1;

%x0_og = [3; 6; 0; 0];
x0_og = [1; 1; 0; 0];
x0 = x0_og;

%% Control loop simulation
% Simulate
tsim = t0;
mpc_sol = [];
x0 = x0_og;

% Compute CEM
cem_controller = CreateCEMControllerBall(tf - t0, x0, BallParams, GuardParams, CostParams);

% Simulate CEM
tsim = t0;
cem_sol = [];
x0 = x0_og;
while tsim < tf
    cem_sol = [cem_sol, BallSimulation(tsim, tf, x0, cem_controller, BallParams, GuardParams)];
    x0 = ResetMap(cem_sol(end), BallParams, GuardParams);
    tsim = cem_sol(end).x(end);
end

%AnimateBall(cem_sol, BallParams, GuardParams);

x0 = x0_og;
domains = ExtractDomainInfo(x0, (tf-t0), cem_controller, BallParams, GuardParams);
domains
terminal = false;
for dom = 1:length(domains.guard_idx)
    if domains.guard_idx(dom) == 3
        terminal = true;
    end
end

if ~terminal
    error("CEM did not find the terminal constraint");
end

warmstart = CreateWarmstart(cem_sol, cem_controller, domains);

tsim = t0;
mpc_sol = [];
x0 = x0_og;
while tsim < tf
    % TODO: Update warm start

    % Compute MPC
    [tmpc, xmpc, umpc, te, xe] = BouncingBallMPC(domains, costfcn, x0, warmstart, BallParams, GuardParams);
    
    % Create MPC controller
    mpc_controller.Compute = @(t, x) MpcController(t, x, umpc, tmpc);

    % Simulate
    mpc_sol = [mpc_sol, BallSimulation(tsim, min(tf, tsim + mpc_dt), x0, mpc_controller, BallParams, GuardParams)];

    % TODO: Plot results (sim and MPC)

    x0 = mpc_sol(end).y(:, end);
    if ~isempty(mpc_sol(end).ie)
        x0 = ResetMap(mpc_sol(end), BallParams, GuardParams);

        % Compute CEM
        cem_controller = CreateCEMControllerBall(tf - t0, x0, BallParams, GuardParams, CostParams);

        domains = ExtractDomainInfo(x0, (tf-t0), cem_controller, BallParams, GuardParams);
        domains
        terminal = false;
        for dom = 1:length(domains.guard_idx)
            if domains.guard_idx(dom) == 3
                terminal = true;
            end
        end
        
        if ~terminal
            error("CEM did not find the terminal constraint");
        end
    else
        if domains.T(1) > mpc_dt
            domains.T(1) = domains.T(1) - mpc_dt;
        else
            diff = 0;
            while domains.T(1) < mpc_dt
                diff = mpc_dt - domains.T(1);
                domains.T(1) = [];
                domains.type(1) = [];
                domains.guard_idx(1) = [];
                domains.variable(1) = [];
                domains.guard_constraint(1) = [];
                domains.guard_val(1) = [];
                domains.nodes(1) = [];
            end

            domains.T(1) = domains.T(1) - diff;
        end
        domains.T(end) = domains.T(end) + mpc_dt;
    end
    tsim = mpc_sol(end).x(end);
end

% Compute MPC

% Compute CEM if a contact is detected


%% Plot
% CEM
%AnimateBall(cem_sol, BallParams, GuardParams);

% MPC
AnimateBall(mpc_sol, BallParams, GuardParams);

%PlotMPCSolutionAndSim(tmpc, xmpc, umpc, mpc_sol);
%domains
%AnimateMPC(tmpc, xmpc, BallParams, GuardParams);

% Both
%AnimateTwoBalls(mpc_sol, cem_sol, BallParams, GuardParams);

%% Controller Functions
function u = ZeroController()
    u = zeros(2, 1);
end

function uout = MpcController(t, x, u, tu)
    uout = interp1(tu(1:end-1), u', t)';
end

%% Plotting
function PlotMPCSolutionAndSim(tmpc, xmpc, umpc, sim_sol)

    tsim = linspace(sim_sol(1).x(1), sim_sol(end).x(end), 200);
    xsim = zeros(4, length(tsim));

    for i = 1:length(tsim)
       
        xsim(:, i) = GetState(sim_sol, tsim(i));
    end

    figure;
    plot(xmpc(1,:), xmpc(2,:), "LineWidth", 2);
    hold on;
    plot(xsim(1, :), xsim(2, :), "LineWidth", 2);
    hold off;

    figure;
    tiledlayout(3, 2);
    nexttile;
    hold on;
    plot(tmpc, xmpc(1, :));
    plot(tsim, xsim(1, :));
    hold off;
    legend(["mpc", "sim"]);
    title("x position")
    nexttile;
    hold on;
    plot(tmpc, xmpc(3, :));
    plot(tsim, xsim(3, :));
    hold off;
    legend(["mpc", "sim"]);
    title("x velocity")
    nexttile;
    hold on;
    plot(tmpc, xmpc(2, :));
    plot(tsim, xsim(2, :));
    hold off;
    legend(["mpc", "sim"]);
    title("y position")
    nexttile;
    hold on;
    plot(tmpc, xmpc(4, :));
    plot(tsim, xsim(4, :));
    hold off;
    legend(["mpc", "sim"]);
    title("y velocity")
    nexttile;
    plot(tmpc(1:end-1), umpc(1,:));
    nexttile;
    plot(tmpc(1:end-1), umpc(2,:));
    

end


function x = GetState(sol, t)
j = 1;
while sol(j).x(end) < t
    j = j + 1;
end

x = deval(sol(j), t);

end

function warm_start = CreateWarmstart(cem_solution, cem_controller, domains)
warm_start.x = zeros(4, sum(domains.nodes));
warm_start.u = zeros(2, sum(domains.nodes));    

t = linspace(0, sum(domains.T), length(warm_start.x));
for i = 1:length(warm_start.x)
    warm_start.x(:, i) = GetState(cem_solution, t(i));
    warm_start.u(:, i) = cem_controller.Compute(t(i), warm_start.x(:, i));
end

end


