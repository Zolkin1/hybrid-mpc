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
swing_params.tf = [0.3; 0.3; 0.3]; %0.3;
swing_params.nodes = [30; 30; 30];
swing_params.length = [0.3; 0.3; 0.3];
swing_params.num_swings = 3;
swing_params.start_pos = ForwardKinematics(robot, q0, v0, robot.swing, robot.foot_r);
swing_params.time_into_swing = 0.0;
swing_params.no_swing_constraint = 4;

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

te(1) = swing_params.tf(1);
for i = 2:swing_params.num_swings
    te(i) = te(i-1) + swing_params.tf(i);
end

foot_pos = zeros(2, length(t));
torso_pos = zeros(2, length(t));
head_pos = zeros(2, length(t));
for i = 1:length(t)
    foot_pos(:, i) = ForwardKinematics(robot, q(:, i), v(:, i), robot.swing, robot.foot_r);
    torso_pos(:, i) = ForwardKinematics(robot, q(:, i), v(:, i), 3, [0; 0]);
    head_pos(:, i) = ForwardKinematics(robot, q(:, i), v(:, i), 3, [0; robot.torso_length]);
end

%% Run the controller on the sim
% TODO: Fill out the controller
controller.p = [7000; 7000; 7000; 7000; 7000];
controller.d = [100; 100; 100; 100; 100];
controller.saturation = [30000 30000 30000 30000 30000];
% controller.q = q;
% controller.v = v;
% controller.t = t;
% controller.u = u;
controller.Compute = @(tc, qc, vc, controller) MpcController(tc, qc, vc, controller);

% % Only used for compliance
controller.q_target = 0;
controller.v_target = 0;
% 
% [tsim, qsim, vsim, base_possim, tesim, qesim, vesim, pos_esim] = ...
%     RobotSim(robot, q0, v0, pos0, t0, tf, 100, controller);

%% Short Horizon MPC
% short_swing_params.apex = 0.1;
% short_swing_params.tf = [0.3];
% short_swing_params.nodes = [30];
% short_swing_params.length = [0.3];
% short_swing_params.num_swings = 1;
% short_swing_params.start_pos = ForwardKinematics(robot, q0, v0, robot.swing, robot.foot_r);
% short_swing_params.time_into_swing = 0.0;
% short_swing_params.no_swing_constraint = 2;

%% Run MPC in the loop
% mpc_dt = 0.1;
% mpc_sim_tf = 0.5;
% 
% MpcFunction = @(q0, v0, pos0, swing_params, warmstart) WalkingMPC(robot, q0, v0, pos0, swing_params, costfcn, warmstart);
% [tmpcsim, qmpcsim, vmpcsim, posmpcsim, tempcsim, qempcsim, vempcsim, pos_empcsim] = ...
%     MpcSim(t0, mpc_sim_tf, q0, v0, pos0, mpc_dt, short_swing_params, MpcFunction, controller, robot);
%% Plots

figure;
plot(t, foot_pos(1,:));
hold on;
plot(t, foot_pos(2, :));
hold off;
legend(["x", "y"])
xlabel("Time (s)")
ylabel("Foot Positions (m)")
% 
% figure;
% plot(t, torso_pos(1, :));
% hold on;
% plot(t, torso_pos(2, :));
% hold off;
% legend(["x", "y"])
% xlabel("Time (s)")
% ylabel("Torso Positions (m)")
% 
% figure;
% plot(t, head_pos(1, :));
% hold on;
% plot(t, head_pos(2, :));
% hold off;
% legend(["x", "y"])
% xlabel("Time (s)")
% ylabel("Head Positions (m)")
% 
% figure;
% hold on;
% for i = 1:robot.nj_act
%     plot(t(1:end-1), u(i,:));
% end
% hold off;
% legend(["u1", "u2", "u3", "u4", "u5"]);
% xlabel("Time (s)")
% ylabel("Torque")
% 
% figure;
% plot(t, q);
% hold on;
% plot(tsim, qsim);
% hold off;
% legend(["qt1", "qt2", "qt3", "qt4", "qt5", "qa1", "qa2", "qa3", "qa4", "qa5"])
% xlabel("Time (s)")
% ylabel("Joints")

%showmotion(robot, t, q);

AnimateRobot(robot, t, q, pos, te, qe, ve, pos_e);

%AnimateRobot(robot, tsim, qsim, base_possim, tesim, qesim, vesim, pos_esim);

%AnimateRobot(robot, tmpcsim, qmpcsim, posmpcsim, tempcsim, qempcsim, vempcsim, pos_empcsim);

%% Cost function
% function cost = ComputeStageCost(x, u, t, curr_foot_pos, swing_num, robot, params)
%     import casadi.*
%     cost = PositionTracking(x(robot.nq + robot.nv + 1: end),...
%         params.pos_target, params.pos_weight);
% 
%     cost = cost + PoseTracking(x(1:robot.nq), params.pose_target,...
%        params.pose_weight);
% 
%     cost = cost + VelocityReg(x(robot.nq + 1:robot.nq + robot.nv),...
%        params.vel_target, params.vel_weight);
% 
%     cost = cost + TorqueReg(u, params.u_target, params.u_weight);
% 
%     cost = cost + FootTracking(x, t, curr_foot_pos, swing_num, params.swing_params, params.swing_weights, robot);
% 
%     cost = cost + HeadTracking(x, params.head_target, params.head_weight, robot);
% 
%     cost = cost*params.dt;
% end
% 
% function cost = PositionTracking(pos, pos_target, pos_weight)
%     cost = dot(pos_weight.*(pos - pos_target), pos_weight.*(pos - pos_target));
% end
% 
% function cost = PoseTracking(q, q_target, q_weight)
%     cost = dot(q_weight.*(q - q_target), q_weight.*(q - q_target));
% end
% 
% function cost = VelocityReg(v, v_target, v_weight)
%     cost = dot(v_weight.*(v - v_target), v_weight.*(v - v_target));
% end
% 
% function cost = TorqueReg(u, u_target, u_weight)
%     cost = dot(u_weight.*(u - u_target), u_weight.*(u - u_target));
% end
% 
% function cost = FootTracking(x, t, curr_foot_pos, swing_num, params, weights, robot)
%     fk_pos = ForwardKinematicsCasadi(robot, x, robot.swing, robot.foot_r);
%     des_swing_pos = SwingTrajectory(t, 0, params.tf(swing_num), curr_foot_pos(1), ...
%                 params.length(swing_num), params.apex, 0, 0);
%     cost = dot(weights.*(fk_pos - des_swing_pos), weights.*(fk_pos - des_swing_pos));
% end
% 
% function cost = HeadTracking(x, target, weight, robot)
%     fk_pos = ForwardKinematicsCasadi(robot, x, 3, [0; robot.torso_length]);
%     cost = dot(weight.*(fk_pos - target), weight.*(fk_pos - target));
% end
% 
function tau = MpcController(tc, qc, vc, controller)
    uff = interp1(controller.t(1:end-1), controller.u', tc)';
    controller.q_target = interp1(controller.t, controller.q', tc)';
    controller.v_target = interp1(controller.t, controller.v', tc)';

    controller.feed_forward = uff; % zeros(length(uff), 1);

    tau = PDController(tc, qc, vc, controller);
end
