clc;
close all;
clear;

% TODO List:
% - Make sure it works in the sim with the PD controller
% - Make it look nicer (tuning or constraints)
% - Figure out how to warm start/speed up the MPC
% - See how the MPC does with disturbances
% - Add in sampling based optimization
% - Merge the two
% - Generate all the plots I need (monte carlo sim)

%%
robot = CreateFiveLink();

q0 = [-0.1; 0.1; -0.2; 0.8; -0.4];
%q0 = [-0.1; -0.1; -0.1; 0.1; 0.1];
v0 = zeros(robot.nv, 1);
robot.torso_pos = ForwardKinematics(robot, q0, v0, 3, [0;0]);
pos0 = robot.torso_pos;

swing_params.apex = 0.1;
swing_params.tf = [0.3; 0.3; 0.3]; %0.3;
swing_params.nodes = [25; 25; 25];
swing_params.length = [0.3; 0.3; 0.3];
swing_params.num_swings = 3;
swing_params.start_pos = ForwardKinematics(robot, q0, v0, robot.swing, robot.foot_r);
swing_params.time_into_swing = 0.1;
swing_params.no_swing_constraint = 4;

cost_params.pose_target = [-0.1; 1.5; 0.2; 0.8; -0.4];
cost_params.pose_weight = [5, 5, 50, 75, 5]'; %[100; 100; 100; 100; 100]; %[0, 0, 0, 0, 0]';

cost_params.vel_target = [0, 0, 0, 0, 0]';
cost_params.vel_weight = [50., 50., 50., 50., 100.]';

cost_params.pos_target = [2, 0.8]';
cost_params.pos_weight = [300, 100]';

cost_params.u_target = [0, 0, 0, 0, 0]';
cost_params.u_weight = [5, 5, 5, 5, 10]';

cost_params.dt = swing_params.tf(1)/swing_params.nodes(1);
cost_params.swing_params = swing_params;
cost_params.swing_weights = [1, 1]';

cost_params.head_target = [2; 1.6];
cost_params.head_weight = [0; 700];

costfcn = @(x, u, t, curr_foot_pos, swing_num) ...
    ComputeStageCost(x, u, t, curr_foot_pos, swing_num, robot, cost_params);

[t, u, q, v, pos, torques, te, qe, ve, pos_e, cost] = ...
    WalkingMPC(robot, q0, v0, pos0, swing_params, costfcn);

% te(1) = swing_params.tf;
% for i = 2:swing_params.num_swings
%     te(i) = te(i-1) + swing_params.tf;
% end

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
%[t, q, qd, base_pos, te, qe, ve, pos_e] = RobotSim(robot, q0, v0, pos0, t0, tf, 20, controller);


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
plot(t, torso_pos(1, :));
hold on;
plot(t, torso_pos(2, :));
hold off;
legend(["x", "y"])
xlabel("Time (s)")
ylabel("Torso Positions (m)")

figure;
plot(t, head_pos(1, :));
hold on;
plot(t, head_pos(2, :));
hold off;
legend(["x", "y"])
xlabel("Time (s)")
ylabel("Head Positions (m)")

figure;
hold on;
for i = 1:robot.nj_act
    plot(t(1:end-1), u(i,:));
end
hold off;
legend(["u1", "u2", "u3", "u4", "u5"]);
xlabel("Time (s)")
ylabel("Torque")

showmotion(robot, t, q);

AnimateRobot(robot, t, q, pos, te, qe, ve, pos_e);

%% Cost function
function cost = ComputeStageCost(x, u, t, curr_foot_pos, swing_num, robot, params)
    import casadi.*
    cost = PositionTracking(x(robot.nq + robot.nv + 1: end),...
        params.pos_target, params.pos_weight);

    cost = cost + PoseTracking(x(1:robot.nq), params.pose_target,...
       params.pose_weight);

    cost = cost + VelocityReg(x(robot.nq + 1:robot.nq + robot.nv),...
       params.vel_target, params.vel_weight);

    cost = cost + TorqueReg(u, params.u_target, params.u_weight);

    cost = cost + FootTracking(x, t, curr_foot_pos, swing_num, params.swing_params, params.swing_weights, robot);

    cost = cost + HeadTracking(x, params.head_target, params.head_weight, robot);

    cost = cost*params.dt;
end

function cost = PositionTracking(pos, pos_target, pos_weight)
    cost = dot(pos_weight.*(pos - pos_target), pos_weight.*(pos - pos_target));
end

function cost = PoseTracking(q, q_target, q_weight)
    cost = dot(q_weight.*(q - q_target), q_weight.*(q - q_target));
end

function cost = VelocityReg(v, v_target, v_weight)
    cost = dot(v_weight.*(v - v_target), v_weight.*(v - v_target));
end

function cost = TorqueReg(u, u_target, u_weight)
    cost = dot(u_weight.*(u - u_target), u_weight.*(u - u_target));
end

function cost = FootTracking(x, t, curr_foot_pos, swing_num, params, weights, robot)
    fk_pos = ForwardKinematicsCasadi(robot, x, robot.swing, robot.foot_r);
    des_swing_pos = SwingTrajectory(t, 0, params.tf(swing_num), curr_foot_pos(1), ...
                params.length(swing_num), params.apex, 0, 0);
    cost = dot(weights.*(fk_pos - des_swing_pos), weights.*(fk_pos - des_swing_pos));
end

function cost = HeadTracking(x, target, weight, robot)
    fk_pos = ForwardKinematicsCasadi(robot, x, 3, [0; robot.torso_length]);
    cost = dot(weight.*(fk_pos - target), weight.*(fk_pos - target));
end
