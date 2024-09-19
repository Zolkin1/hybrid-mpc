clc;
clear;
close all;

%% Robot
robot = CreateFiveLink();

%% Initial Condition
%q0 = [-0.1; 0.1; -0.2; 0.8; -0.4];
%q0 = [-0.1; -0.1; -0.1; 0.1; 0.1];
q0 = [-0.78; 0.9; 0.0133; 0.51; -1.0951];
v0 = zeros(robot.nv, 1);
robot.torso_pos = ForwardKinematics(robot, q0, v0, 3, [0;0]);
pos0 = robot.torso_pos;

t0 = 0;

%% Controller
controller.p = [7000; 7000; 7000; 7000; 7000];
controller.d = [100; 100; 100; 100; 100];
controller.saturation = [30000 30000 30000 30000 30000];
controller.Compute = @(tc, qc, vc, controller) MpcController(tc, qc, vc, controller);

%% Cost Function
cost_params.pose_target = [-0.1; 1.5; 0.2; 0.8; -0.4];
cost_params.pose_weight = [5, 5, 5, 5, 5]';
%cost_params.pose_weight = [1000; 1000; 1000; 1000; 1000];

cost_params.vel_target = [0, 0, 0, 0, 0]';
cost_params.vel_weight = [50., 500., 50., 500., 500.]';

cost_params.pos_target = [2, 0.8]';
cost_params.pos_weight = [300, 1000]';

cost_params.u_target = [0, 0, 0, 0, 0]';
cost_params.u_weight = [0.001, 1, 0.001, 1, 1]';

cost_params.swing_weights = [1, 1]';

cost_params.head_target = [2; 1.6];
cost_params.head_weight = [0; 7000];

costfcn = @(x, u, t, dt, swing_num) ...
    ComputeStageCost(x, u, t, dt, robot, cost_params);

%% Short Horizon MPC
short_swing_params.apex = 0.1;
short_swing_params.tf = [0.3];
short_swing_params.nodes = [30];
short_swing_params.length = [0.3];
short_swing_params.num_swings = 1;
short_swing_params.start_pos = ForwardKinematics(robot, q0, v0, robot.swing, robot.foot_r);
short_swing_params.time_into_swing = 0.0;
short_swing_params.no_swing_constraint = 2;

%% Run MPC in the loop
mpc_dt = 0.15;
mpc_sim_tf = 1.2;

MpcFunction = @(q0, v0, pos0, swing_params, warmstart) WalkingMPC(robot, q0, v0, pos0, swing_params, costfcn, warmstart);
[tmpcsim, qmpcsim, vmpcsim, posmpcsim, tempcsim, qempcsim, vempcsim, pos_empcsim] = ...
    MpcSim(t0, mpc_sim_tf, q0, v0, pos0, mpc_dt, short_swing_params, MpcFunction, controller, robot);

%% Animation and Plots
AnimateRobot(robot, tmpcsim, qmpcsim, posmpcsim, tempcsim, qempcsim, vempcsim, pos_empcsim);


swing_foot_pos = zeros(2, length(tmpcsim));
for i = 1:length(tmpcsim)
    swing_foot_pos(:, i) = ForwardKinematics(robot, qmpcsim(:, i), vmpcsim(:, i), robot.swing, robot.foot_r);
end

figure;
plot(tmpcsim, swing_foot_pos(1,:));
hold on;
plot(tmpcsim, swing_foot_pos(2, :));
hold off;
xlabel("Time (s)")
ylabel("Foot Position")
legend(["x", "y"])
title("Swing Foot Position")

%% Controller Functions
function tau = MpcController(tc, qc, vc, controller)
    uff = interp1(controller.t(1:end-1), controller.u', tc)';
    controller.q_target = interp1(controller.t, controller.q', tc)';
    controller.v_target = interp1(controller.t, controller.v', tc)';

    controller.feed_forward = uff; % zeros(length(uff), 1);

    tau = PDController(tc, qc, vc, controller);
end