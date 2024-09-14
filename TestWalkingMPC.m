clc;
close all;
clear;

%%
robot = CreateFiveLink();

q0 = [-0.1; 0.1; -0.2; 0.8; -0.1];
v0 = zeros(robot.nv, 1);
robot.torso_pos = ForwardKinematics(robot, q0, v0, 3, [0;0]);
pos0 = robot.torso_pos;

swing_params.apex = 0.1;
swing_params.tf = 0.3;
swing_params.nodes = 25;
swing_params.length = 0.3;
swing_params.num_swings = 1;

cost_params.pose_target = q0;
cost_params.pose_weight = [0, 0, 0, 0, 0]';

cost_params.vel_target = [0, 0, 0, 0, 0]';
cost_params.vel_weight = [0., 0., 0., 0., 0.]';

cost_params.pos_target = [2, 0.7]';
cost_params.pos_weight = [100, 200]';

cost_params.u_target = [0, 0, 0, 0]';
cost_params.u_weight = [100, 100, 100, 100]';

cost_params.dt = swing_params.tf/swing_params.nodes;

costfcn = @(x, u) ComputeStageCost(x, u, robot, cost_params);

[t, q, v, pos, torques, qe, ve, pos_e, cost] = WalkingMPC(robot, q0, v0, pos0, swing_params, costfcn);

te(1) = swing_params.tf;
for i = 2:swing_params.num_swings
    te(i) = te(i-1) + swing_params.tf;
end

AnimateRobot(robot, t, q, pos, te, qe, ve, pos_e);

%% Cost function
function cost = ComputeStageCost(x, u, robot, params)
    import casadi.*
    cost = PositionTracking(x(robot.nq + robot.nv + 1: end),...
        params.pos_target, params.pos_weight);
    cost = cost + PoseTracking(x(1:robot.nq), params.pose_target,...
       params.pose_weight);
    cost = cost + VelocityReg(x(robot.nq + 1:robot.nq + robot.nv),...
       params.vel_target, params.vel_weight);
    cost = cost + TorqueReg(u, params.u_target, params.u_weight);

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