clc;
close all;
clear;

%%
robot = CreateFiveLink();

%q0 = [-0.1; 0.1; -0.2; 0.8; -0.4];
%q0 = [-0.1; -0.1; -0.1; 0.1; 0.1];
q0 = [-0.78; 0.9; 0.0133; 0.51; -1.0951];
v0 = zeros(robot.nv, 1);
robot.torso_pos = ForwardKinematics(robot, q0, v0, 3, [0;0]);
pos0 = robot.torso_pos;

t0 = 0;

% Generate the sample
xsample = [0.3, 0.3, 0.3, 0.3, 0.3, 0.2, 0.4, 0.5];

% Run CEM
%cost = CemCostFunction(xsample, q0, v0, pos0, robot);
%cost

num_swings = 5;

costfcn = @(xsample) CemCostFunction(xsample, q0, v0, pos0, num_swings, robot);

cem_settings.nsamples = 10;
cem_settings.xsize = 2*num_swings;
cem_settings.var = 0.25*eye(cem_settings.xsize);
cem_settings.mean = 0.3*ones(cem_settings.xsize, 1);
cem_settings.nelite = 3;
cem_settings.max_iters = 3;

rng("default")
[xmean, xvar, yopt] = CrossEntropyOptimization(cem_settings, costfcn);

%% Plots
AnimateRobot(robot, t, q, pos, te, qe, ve, pos_e);

%% Functions
function cost = ...
    CemCostFunction(xsample, q0, v0, pos0, num_swings, robot)

    for i = 1:length(xsample)
        xsample(i) = max(0, xsample(i));
    end

    swing_params.num_swings = num_swings;
    swing_params.start_pos = ForwardKinematics(robot, q0, v0, robot.swing, robot.foot_r);
    swing_params.time_into_swing = 0.0;
    swing_params.no_swing_constraint = 100;
    swing_params.apex = 0.1;
    swing_params.final_height = 0;

    vars_per_swing = 2;

    swing_params.tf = zeros(swing_params.num_swings, 1);
    swing_params.length = zeros(swing_params.num_swings, 1);

    for i = 1:swing_params.num_swings
        % Long Horizon MPC
        swing_params.tf(i) = xsample(1 + (i-1)*vars_per_swing);
        swing_params.nodes(i) = ceil(swing_params.tf(i)*100);
        swing_params.length(i) = xsample(2 + (i-1)*vars_per_swing);
    end
    
    tf = sum(swing_params.tf);
    
    cost_params.pose_target = [-0.1; 1.5; 0.2; 0.8; -0.4];
    cost_params.pose_weight = [5, 5, 5, 5, 5]';
    %cost_params.pose_weight = [1000; 1000; 1000; 1000; 1000];
    
    cost_params.vel_target = [0, 0, 0, 0, 0]';
    cost_params.vel_weight = [50., 500., 50., 500., 500.]';
    
    cost_params.pos_target = [2, 0.8]';
    cost_params.pos_weight = [300, 1000]';
    
    cost_params.u_target = [0, 0, 0, 0, 0]';
    cost_params.u_weight = [0.001, 1, 0.001, 1, 1]';
    
    cost_params.dt = swing_params.tf(1)/swing_params.nodes(1);
    cost_params.swing_params = swing_params;
    cost_params.swing_weights = [1, 1]';
    
    cost_params.head_target = [2; 1.6];
    cost_params.head_weight = [0; 7000];
    
    costfcn = @(x, u, t, dt) ...
        ComputeStageCost(x, u, t, dt, robot, cost_params);
    
    total_nodes = sum(swing_params.nodes);
    warmstart.q = zeros(robot.nq, total_nodes);
    warmstart.v = zeros(robot.nv, total_nodes);
    warmstart.pos = zeros(2, total_nodes);
    warmstart.tau = zeros(robot.nj_act, total_nodes);
    
    for node = 1:total_nodes
        warmstart.q(:, node) = q0;
        warmstart.v(:, node) = v0;
        warmstart.pos(:, node) = pos0;
        warmstart.tau(:, node) = zeros(robot.nj_act, 1);
    end
    
    [t, u, q, v, pos, te, qe, ve, pos_e, cost] = ...
       WalkingMPC(robot, q0, v0, pos0, swing_params, costfcn, warmstart);
end

function tau = MpcController(tc, qc, vc, controller)
    uff = interp1(controller.t(1:end-1), controller.u', tc)';
    controller.q_target = interp1(controller.t, controller.q', tc)';
    controller.v_target = interp1(controller.t, controller.v', tc)';

    controller.feed_forward = uff; % zeros(length(uff), 1);

    tau = PDController(tc, qc, vc, controller);
end
