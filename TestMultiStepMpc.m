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

%% Long Horizon MPC
swing_params.apex = 0.1;
swing_params.num_swings = 6;
swing_params.nodes = 30*ones(swing_params.num_swings, 1);
swing_params.start_pos = ForwardKinematics(robot, q0, v0, robot.swing, robot.foot_r);
swing_params.time_into_swing = 0.0;
swing_params.no_swing_constraint = swing_params.num_swings + 1;
swing_params.final_height = 0;

cost_params.pose_target = [-0.1; 1.5; 0.2; 0.8; -0.4];
cost_params.pose_weight = [5, 5, 5, 5, 5]';
%cost_params.pose_weight = [1000; 1000; 1000; 1000; 1000];

cost_params.vel_target = [0, 0, 0, 0, 0]';
cost_params.vel_weight = [5., 25., 25., 25., 5.]';

cost_params.pos_target = [2, 0.8]';
cost_params.pos_weight = [3000, 1000]';

cost_params.u_target = [0, 0, 0, 0, 0]';
cost_params.u_weight = [10, 10, 10, 10, 10]';

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

mpc_sol = MultiStepMPC(robot, q0, v0, pos0, swing_params, costfcn, warmstart);

u = mpc_sol.u;
q = mpc_sol.q;
v = mpc_sol.v;
t = mpc_sol.t;
pos = mpc_sol.pos;

te = mpc_sol.te;
qe = mpc_sol.qe;
ve = mpc_sol.ve;
pos_e = mpc_sol.pos_e;

foot_pos = zeros(2, length(t));
torso_pos = zeros(2, length(t));
head_pos = zeros(2, length(t));
for i = 1:length(t)
    foot_pos(:, i) = ForwardKinematics(robot, q(:, i), v(:, i), robot.swing, robot.foot_r);
    torso_pos(:, i) = ForwardKinematics(robot, q(:, i), v(:, i), 3, [0; 0]);
    head_pos(:, i) = ForwardKinematics(robot, q(:, i), v(:, i), 3, [0; robot.torso_length]);
end

%% Run the controller on the sim
controller.p = [7000; 7000; 7000; 7000; 7000];
controller.d = [100; 100; 100; 100; 100];
controller.saturation = [30000 30000 30000 30000 30000];
controller.Compute = @(tc, qc, vc, controller) MpcController(tc, qc, vc, controller);

% Only used for compliance
controller.q_target = 0;
controller.v_target = 0;
%% Plots

figure;
plot(t, foot_pos(1,:));
hold on;
plot(t, foot_pos(2, :));
hold off;
legend(["x", "y"])
xlabel("Time (s)")
ylabel("Foot Positions (m)")

figure;
plot(t(1:end-1), u);
legend(["u1", "u2", "u3", "u4"]);
xlabel("Time (s)")
ylabel("Input")


AnimateRobot(robot, t, q, pos, te, qe, ve, pos_e);
%% Functions
function tau = MpcController(tc, qc, vc, controller)
    uff = interp1(controller.t(1:end-1), controller.u', tc)';
    controller.q_target = interp1(controller.t, controller.q', tc)';
    controller.v_target = interp1(controller.t, controller.v', tc)';

    controller.feed_forward = uff; % zeros(length(uff), 1);

    tau = PDController(tc, qc, vc, controller);
end
