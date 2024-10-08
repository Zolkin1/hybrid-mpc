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
% GuardParams.G{3} = [4, 2; 6, 2];
% GuardParams.G{4} = [4, 2; 4, 4];
% GuardParams.G{5} = [6, 2; 6, 4];
% GuardParams.G{6} = [4, 4; 6, 4];

GuardParams.Color(1) = "k";
GuardParams.Color(2) = "k";
GuardParams.Color(3) = "b";
% GuardParams.Color(4) = "b";
% GuardParams.Color(5) = "b";
% GuardParams.Color(6) = "b";

GuardParams.Direction{1} = [-1, 2, 0, 8]; % inc.dec value, x/y constraint, start value, end value
GuardParams.Direction{2} = [-1, 1, 0, 8];
GuardParams.Direction{3} = [0, 1, 2, 4];
% GuardParams.Direction{4} = [0, 2, 4, 6];
% GuardParams.Direction{5} = [0, 2, 4, 6];
% GuardParams.Direction{6} = [0, 1, 2, 4];

GuardParams.Value(1) = 0;
GuardParams.Value(2) = 0;
GuardParams.Value(3) = 2;
% GuardParams.Value(4) = 4;
% GuardParams.Value(5) = 6;
% GuardParams.Value(6) = 4;

%%
t0 = 0;
tf = 5;

controller.Compute = @(t, x) ZeroController();

%x0_og = [3; 6; 0; 0];
x0_og = [1; 1; 0; 0];
x0 = x0_og;
%% CEM
CostParams.position_target = [3; 3];
CostParams.position_weight = [50; 50];
CostParams.velocity_target = [0; 0];
CostParams.velocity_weight = 0.001*[1; 1];
CostParams.u_target = [0; 0];

CostParams.u_weight = 0.001*[1; 1];
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

%% MPC
x0 = x0_og;
domains = ExtractDomainInfo(x0, (tf-t0), cem_controller, BallParams, GuardParams);
domains

costfcn = @(x, u, dt) BallStageCost(x, u, dt, CostParams);

warmstart = CreateWarmstart(cem_sol, cem_controller, domains);

[tmpc, xmpc, umpc, te, xe] = BouncingBallMPC(domains, costfcn, x0, warmstart, BallParams, GuardParams);

% Create MPC controller

mpc_controller.Compute = @(t, x) MpcController(t, x, umpc, tmpc);

% MPC
tsim = t0;
mpc_sol = [];
x0 = x0_og;
while tsim < tf
    mpc_sol = [mpc_sol, BallSimulation(tsim, tf, x0, mpc_controller, BallParams, GuardParams)];
    x0 = ResetMap(mpc_sol(end), BallParams, GuardParams);
    tsim = mpc_sol(end).x(end);
end

%% Plot
% CEM
%AnimateBall(cem_sol, BallParams, GuardParams);

% MPC
%AnimateBall(mpc_sol, BallParams, GuardParams);

PlotMPCSolutionAndSim(tmpc, xmpc, umpc, mpc_sol);
%domains
%AnimateMPC(tmpc, xmpc, BallParams, GuardParams);

% Both
AnimateTwoBalls(mpc_sol, cem_sol, BallParams, GuardParams);

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


