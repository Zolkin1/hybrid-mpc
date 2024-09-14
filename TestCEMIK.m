clc;
clear;
close all;

%%
robot = CreateFiveLink();

q0 = [-0.1; 0.1; -0.2; -0.1; -0.1];
v0 = [0; 0; 0; 0; 0];
robot.torso_pos = ForwardKinematics(robot, q0, v0, 3, [0;0]);
pos0 = robot.torso_pos;

t0 = 0;
tf = 3;
tfswing = 3;
times = linspace(0, 3, 300);

variables_per_swing = 2;
num_swings = 9;

cem_settings.nsamples = 10;
cem_settings.xsize = num_swings*variables_per_swing + 1;
cem_settings.var = 0.125*eye(cem_settings.xsize); % 0.125
cem_settings.mean = 0.1*ones(cem_settings.xsize, 1);
cem_settings.nelite = 1;
cem_settings.max_iters = 3;

cost_params.pose_target = q0; %[0, 0, 0, 0.1, -0.1];
cost_params.pose_weight = [0, 0, 0, 0, 0];

cost_params.vel_target = [0, 0, 0, 0, 0];
cost_params.vel_weight = [0., 0., 0., 0., 0.];

cost_params.pos_target = [2, 0.7];
cost_params.pos_weight = [100; 200]; %[10, 5];

costfcn = @(samples) CostFcn(times, robot, q0, v0, pos0, cost_params, samples);
[cemean, cevar, yopt] = CrossEntropyOptimization(cem_settings, costfcn);

controller = CreateController(robot, q0, cemean);
[t, q, qd, base_pos, te, qe, ve, pos_e] = RobotSim(robot, q0, v0, pos0, t0, tf, 20, controller);


%AnimateRobot(robot, t, q, base_pos, te, qe, ve, pos_e);


%% Cost function
function cost = CostFcn(times, robot, q0, v0, pos0, cost_params, samplevec)
    controller = CreateController(robot, q0, samplevec);

    % Simulate the robot
    t0 = times(1);
    tf = times(end);
    [t, q, qd, pos, te, qe, qde] = RobotSim(robot, q0, v0, pos0, t0, tf, 20, controller);
    
    % Compute the cost
    dt = times(2) - times(1);
    cost = dt*ComputeCost(t, q, qd, pos, cost_params);
end

function controller = CreateController(robot, q0, samplevec)
    % Compute all the targets
    t0 = 0;
    tf = 3;
    tfswing = 0.3; %samplevec(1); %0.3;
    num_swings = 9;
    t_sample = linspace(t0, tfswing, 100);
    %controller.ttargets = t_sample;
    
    controller.qtargets = zeros(robot.nj_act, num_swings*length(t_sample));
    controller.qtargets(:, 1) = GetActuatedCoords(q0, robot);
    qfull = zeros(robot.nq, num_swings*length(t_sample) + 1);
    qfull(:, 1) = q0;

    controller.vtargets = zeros(robot.nj_act, num_swings*length(t_sample));
    
    controller.ttargets = linspace(t0, tf, num_swings*length(t_sample));

    for j = 1:num_swings
        apex = samplevec((j-1)*2 + 2);
        step_length = samplevec((j-1)*2 + 3);
        for i = 1:length(t_sample)
            pos = SwingTrajectory(t_sample(i), t0, tfswing, 0, step_length, apex, 0, 0);
            FD_DELTA = 1e-8;
            pos_pert = SwingTrajectory(t_sample(i) + FD_DELTA, t0, tfswing, 0, step_length, apex, 0, 0);
            vel = (pos_pert - pos)/FD_DELTA;
            
            [qfull(:, (j-1)*length(t_sample) + i+1), cost] = InverseKinematicsRobot(robot, robot.swing, robot.foot_r, qfull(:,(j-1)*length(t_sample) + i), pos);
            %cost
            controller.qtargets(:,(j-1)*length(t_sample) + i) = GetActuatedCoords(qfull(:, (j-1)*length(t_sample) + i), robot);
            controller.vtargets(:,(j-1)*length(t_sample) + i) = GetActuatedCoords(pinv(BodyJacobianFD(robot, qfull(:,(j-1)*length(t_sample) + i+1), robot.swing, robot.foot_r))*vel, robot);
        end
    end

    % Note: These are NOT used, they are merely for sim compliance
    controller.q_target = zeros(2,1);
    controller.v_target = zeros(2,1);
    
    controller.p = [1000 1000 1000 1000]';
    controller.d = [120 120 120 120]';
    controller.saturation = [3000 3000 3000 3000];
    controller.des_pos = [0.4; 0.4];

    controller.Compute = @(t, q, v, controller) ComputeIKControl(t, q, v, controller, robot);
end

function torques = ComputeIKControl(t, q, v, controller, robot)
    qact = GetActuatedCoords(q, robot);
    vact = GetActuatedCoords(v, robot);

    controller.q_target = interp1(controller.ttargets, controller.qtargets', t)';
    controller.v_target = interp1(controller.ttargets, controller.vtargets', t)';

    torques = PDController(t, qact, vact, controller);
end


function tau = PositionController(t, q, v, times, positions, controller)
    q_target = interp1(times, positions, t)';
    %q_target = [0.8; 0; 0.6; 0];
    FD_DELTA = 1e-8;
    v_target = (interp1(times, positions, t + FD_DELTA)' - q_target)/FD_DELTA;
    %v_target = [0.; 0; 0.; 0];

    q_error = q_target - q;
    v_error = v_target - v;
    
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
