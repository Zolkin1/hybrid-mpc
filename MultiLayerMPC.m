clc;
clear;
close all;

% TODO List:
% - See how the MPC does with disturbances
% - Add in sampling based optimization
% - Merge the two
% - Generate all the plots I need (monte carlo sim)

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
controller.p = [1200; 1200; 1200; 1200; 1200];
controller.d = [40; 40; 40; 40; 40];
controller.saturation = robot.torque_lims;
controller.Compute = @(tc, qc, vc, controller) MpcController(tc, qc, vc, controller);

%% Long Horizon MPC
% Swing Parameters
long_swing_params.apex = 0.1;
long_swing_params.num_swings = 3; %6;
long_swing_params.nodes = 30*ones(long_swing_params.num_swings, 1);
long_swing_params.start_pos = ForwardKinematics(robot, q0, v0, robot.swing, robot.foot_r);
long_swing_params.time_into_swing = 0.0;
long_swing_params.no_swing_constraint = long_swing_params.num_swings + 1;
long_swing_params.final_height = 0;

% Cost Parameters
long_cost_params.pose_target = [-0.1; 1.5; 0.2; 0.8; -0.4];
long_cost_params.pose_weight = [5, 5, 5, 5, 5]';
%cost_params.pose_weight = [1000; 1000; 1000; 1000; 1000];

long_cost_params.vel_target = [0, 0, 0, 0, 0]';
long_cost_params.vel_weight = [5., 25., 25., 25., 5.]';

long_cost_params.pos_target = [2, 0.8]';
long_cost_params.pos_weight = [3000, 1000]';

long_cost_params.u_target = [0, 0, 0, 0, 0]';
long_cost_params.u_weight = [10, 10, 10, 10, 10]';

long_cost_params.swing_params = long_swing_params;
long_cost_params.swing_weights = [1, 1]';

long_cost_params.head_target = [2; 1.6];
long_cost_params.head_weight = [0; 7000];

long_costfcn = @(x, u, t, dt) ...
    ComputeStageCost(x, u, t, dt, robot, long_cost_params);

%% Short Horizon MPC
% Swing Parameters
short_swing_params.apex = 0.1;
short_swing_params.tf = [0.25]; % This is actually the planning horizon
short_swing_params.nodes = [30];
short_swing_params.length = [0.2];
short_swing_params.num_swings = 1;
short_swing_params.start_pos = ForwardKinematics(robot, q0, v0, robot.swing, robot.foot_r);
short_swing_params.time_into_swing = 0.0;
short_swing_params.no_swing_constraint = 2;
short_swing_params.final_height = 0;

% Cost Parameters
short_cost_params.pose_target = [-0.1; 0; 0.2; 0.8; -0.4];
short_cost_params.q_weight = [500, 500, 500, 500, 500]';
%cost_params.pose_weight = [1000; 1000; 1000; 1000; 1000];

short_cost_params.vel_target = [0, 0, 0, 0, 0]';
short_cost_params.v_weight = [500., 500., 500., 500., 500.]';

short_cost_params.pos_target = [1, 0.8]';
short_cost_params.pos_weight = [300, 1000]';

short_cost_params.u_target = [0, 0, 0, 0, 0]';
short_cost_params.u_weight = 0*[0.001, 1, 0.001, 1, 1]';

short_cost_params.swing_weights = [1, 1]';

short_cost_params.head_target = [2; 1.6];
short_cost_params.head_weight = [0; 70];

short_costfcn = @(x, u, t, dt, swing_num) ...
    ComputeStageCost(x, u, t, dt, robot, short_cost_params);

%% Run MPC in the loop
mpc_dt = 0.1; %0.15;
mpc_sim_tf = 1;

MpcFunction = @(q0, v0, pos0, swing_params, warmstart, cost_fcn) SingleStepMPC(robot, q0, v0, pos0, swing_params, cost_fcn, warmstart);
LongMpcFunction = @(q0, v0, pos0, swing_params, warmstart, cost_fcn) MultiStepMPC(robot, q0, v0, pos0, swing_params, cost_fcn, warmstart);

short_mpc.swing_params = short_swing_params;
short_mpc.mpc = MpcFunction;
short_mpc.cost_fcn = short_costfcn;
short_mpc.cost_params = short_cost_params;

long_mpc.swing_params = long_swing_params;
long_mpc.mpc = LongMpcFunction;
long_mpc.cost_fcn = long_costfcn;
long_mpc.cost_params = long_cost_params;

sim_sol = MpcSim(t0, mpc_sim_tf, q0, v0, pos0, mpc_dt, short_mpc, long_mpc, controller, robot);

%% Animation and Plots
AnimateRobot(robot, sim_sol.t, sim_sol.q, sim_sol.pos, sim_sol.te, sim_sol.qe, sim_sol.ve, sim_sol.pos_e);

swing_foot_pos = zeros(2, length(sim_sol.t));
for i = 1:length(sim_sol.t)
    swing_foot_pos(:, i) = ForwardKinematics(robot, sim_sol.q(:, i), sim_sol.v(:, i), robot.swing, robot.foot_r);
end

figure;
plot(sim_sol.t, swing_foot_pos(1,:));
hold on;
plot(sim_sol.t, swing_foot_pos(2, :));
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


