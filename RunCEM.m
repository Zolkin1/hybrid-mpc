clc;
clear;
close all;

% TODO:
% - Fix pivot point velocity at impact
% - Should variance matrix be diagonal?

%% Run CEM
robot = CreateFiveLink();

q0 = [-0.1; 0.1; -0.2; 0.8; -0.1];
% q0 = zeros(robot.nq, 1);
% q0(1) = 0.0;
% q0(2) = 0.2; % 0.8
% q0(4) = 0.8; % 0.6
% q0(5) = -0.3;
% q0(1) = 0.0;
% q0(2) = 0.8; % 0.8
% q0(4) = 0.6; % 0.6
% q0(5) = 0;
v0 = zeros(robot.nv, 1);

t0 = 0;
tf = 1.5;

q_target = GetActuatedCoords(q0, robot); %(3:2+robot.nj_act);
q_target(2) = q_target(2) + 0.5;
v_target = GetActuatedCoords(v0, robot); %(3:2+robot.nj_act);

robot.torso_pos = ForwardKinematics(robot, q0, v0, 3, [0;0]);
pos0 = robot.torso_pos;

% cem_settings.max_iters = 4;
% cem_settings.var = 10;
% cem_settings.samples = 100;
% cem_settings.dt = 0.02;
% cem_settings.terminal_eps = -0.001; %0.1;
% cem_settings.num_elite = 2;
% cem_settings.min_var = 0;
% 
% cost_params.pose_target = q0; %[0, 0, 0, 0.1, -0.1];
% cost_params.pose_weight = [100, 100, 50, 10, 50];
% 
% cost_params.vel_target = [0, 0, 0, 0, 0];
% cost_params.vel_weight = [0.1, 0.1, 0.1, 0.1, 0.1];
% 
% cost_params.pos_target = [2, 0.7];
% cost_params.pos_weight = [0; 0]; %[10, 5];
% 
% costfcn.ComputeCost = @(t, q, v, pos)ComputeCost(t, q, v, pos, cost_params);
% 
% controller = CrossEntropyRobot(robot, costfcn, q0, v0, pos0, tf, cem_settings);

dt = 0.1;
times = t0:dt:tf;
ntimes = length(times);

cem_settings.nsamples = 300;
cem_settings.xsize = ntimes*robot.nj_act;
cem_settings.var = 1.125*eye(cem_settings.xsize); % 0.125
cem_settings.mean = 0*ones(cem_settings.xsize, 1);
cem_settings.nelite = 5;
cem_settings.max_iters = 5;

cost_params.pose_target = q0; %[0, 0, 0, 0.1, -0.1];
cost_params.pose_weight = [0, 0, 0, 0, 0];

cost_params.vel_target = [0, 0, 0, 0, 0];
cost_params.vel_weight = [0., 0., 0., 0., 0.];

cost_params.pos_target = [2, 0.7];
cost_params.pos_weight = [100; 200]; %[10, 5];

costfcn = @(samples) CostFcn(times, robot, q0, v0, pos0, cost_params, samples);
[cemean, cevar, yopt] = CrossEntropyOptimization(cem_settings, costfcn);

% Create the controller
torquesmat = reshape(cemean, [length(times), robot.nj_act]);
controller.p = [50 50 50 50]';
controller.d = [5 5 5 5]';
controller.saturation = [3000 3000 3000 3000];
controller.Compute = @(t,q, v, controller)...
    PositionController(t, q, v, times, torquesmat, controller, robot);
controller.q_target = q0;
controller.v_target = v0;
% controller.Compute = @(t,q, v, controller) interp1(times, torquesmat, t);
% controller.q_target = q0;
% controller.v_target = v0;

[t, q, qd, pos, te, qe, qde, pos_e] = RobotSim(robot, q0, v0, pos0, 0, tf, 100, controller);
q = q(:,1:end-1);
t = t(1:end-1);
showmotion(robot, t, q);

AnimateRobot(robot, t, q, pos, te, qe, qde, pos_e);

figure;
plot(t, pos(1, 1:end-1));
hold on;
plot(t, pos(2, 1:end-1));
hold off;
xlabel("Time")
ylabel("torso pos")
legend(["x", "y"]);

%% Cost function
function cost = CostFcn(times, robot, q0, v0, pos0, cost_params, torquesvec)
    % Create the controller
    torquesmat = reshape(torquesvec, [length(times), robot.nj_act]);
    controller.p = [50 50 50 50]';
    controller.d = [5 5 5 5]';
    controller.saturation = [3000 3000 3000 3000];
    controller.Compute = @(t,q, v, controller)...
        PositionController(t, q, v, times, torquesmat, controller, robot);

    % Note that these are only for compliance with the sim
    controller.q_target = q0;
    controller.v_target = v0;

    % Simulate the robot
    t0 = times(1);
    tf = times(end);
    [t, q, qd, pos, te, qe, qde] = RobotSim(robot, q0, v0, pos0, t0, tf, 20, controller);
    
    % Compute the cost
    dt = times(2) - times(1);
    cost = dt*ComputeCost(t, q, qd, pos, cost_params);
end

function tau = PositionController(t, q, v, times, positions, controller, robot)
    q_target = interp1(times, positions, t)';
    %q_target = [0.8; 0; 0.6; 0];
    FD_DELTA = 1e-8;
    v_target = (interp1(times, positions, t + FD_DELTA)' - q_target)/FD_DELTA;
    %v_target = [0.; 0; 0.; 0];

    q_error = q_target - GetActuatedCoords(q, robot);
    v_error = v_target - GetActuatedCoords(v, robot);
    
    tau = controller.p.*q_error + controller.d.*v_error;
    for i = 1:length(tau)
        tau(i) = min(controller.saturation(i), max(tau(i), -controller.saturation(i)));
    end
end

function cost = ComputeCost(t, q, v, pos, params)
    cost = 0;
    for i = 1:size(q, 2)
        cost = cost + PositionTracking(pos(:,i), params.pos_target, params.pos_weight);
        cost = cost + PoseTracking(q(:, i), params.pose_target, params.pose_weight);
        cost = cost + VelocityReg(v(:,i), params.vel_target, params.vel_weight);
    end
end

function cost = PositionTracking(pos, pos_target, pos_weight)
    cost = norm(pos_weight.*(pos - pos_target));
end

function cost = PoseTracking(q, q_target, q_weight)
    cost = norm(q_weight*(q - q_target));
end

function cost = VelocityReg(v, v_target, v_weight)
    cost = norm(v_weight.*(v - v_target));
end
