clc;
clear;
close all;

%%
BallParams.M = 1;
BallParams.r = 0.1;
BallParams.g = [0; -9.81];
BallParams.gamma = 1; % 0.8

BallParams.ForceLim = [100; 100];

pos0 = [1, 1];


GuardParams.num = 6;
GuardParams.G = zeros(GuardParams.num, 2, 2);
GuardParams.G(1, :, :) = [0, 0; 8, 0];
GuardParams.G(2, :, :) = [0, 0; 0, 8];
GuardParams.G(3, :, :) = [4, 2; 6, 2];
GuardParams.G(4, :, :) = [4, 2; 4, 4];
GuardParams.G(5, :, :) = [6, 2; 6, 4];
GuardParams.G(6, :, :) = [4, 4; 6, 4];

GuardParams.Color(1) = "k";
GuardParams.Color(2) = "k";
GuardParams.Color(3) = "b";
GuardParams.Color(4) = "b";
GuardParams.Color(5) = "b";
GuardParams.Color(6) = "b";

GuardParams.Direction{1} = [-1, 1, 0, 10];
GuardParams.Direction{2} = [-1, 2, 0, 10];
GuardParams.Direction{3} = [0, 2, 2, 4];
GuardParams.Direction{4} = [0, 1, 4, 6];
GuardParams.Direction{5} = [0, 1, 4, 6];
GuardParams.Direction{6} = [0, 2, 2, 4];

GuardParams.Value(1) = 0;
GuardParams.Value(2) = 0;
GuardParams.Value(3) = 2;
GuardParams.Value(4) = 4;
GuardParams.Value(5) = 6;
GuardParams.Value(6) = 4;

%%
t0 = 0;
tf = 5;

controller.Compute = @(t, x) ZeroController();

x0 = [1; 1; 0; 0];

%% CEM
CostParams.position_target = [5; 3];
CostParams.position_weight = [50; 50];
CostParams.velocity_target = [0; 0];
CostParams.velocity_weight = 0.001*[1; 1];
CostParams.u_target = [0; 0];
CostParams.u_weight = 0.001*[1; 1];
cem_controller = CreateCEMControllerBall(tf - t0, x0, BallParams, GuardParams, CostParams);

%% MPC
domains = ExtractDomainInfo(x0, (tf-t0), cem_controller, BallParams, GuardParams);

% TODO: Create cost function
[t, x, u, te, xe] = BouncingBallMPC(domains, costfcn, x0, BallParams, GuardParams);

%% Simulate
tsim = t0;
sol = [];
while tsim < tf
    sol = [sol, BallSimulation(tsim, tf, x0, cem_controller, BallParams, GuardParams)];
    x0 = ResetMap(sol(end), BallParams, GuardParams);
    tsim = sol(end).x(end);
end


%% Plot
AnimateBall(sol, BallParams, GuardParams);

%% Controller Functions
function u = ZeroController()
    u = zeros(2, 1);
end


