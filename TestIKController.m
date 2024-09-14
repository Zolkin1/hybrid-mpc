clc;
close all;
clear;

%%
robot = CreateFiveLink();

q0 = [-0.1; 0.1; -0.2; -0.1; -0.1];
v0 = [0; 0; 0; 0; 0];
robot.torso_pos = ForwardKinematics(robot, q0, v0, 3, [0;0]);
pos0 = robot.torso_pos;

t0 = 0;
tf = 3;
tfswing = 3;

t_sample = linspace(t0, tfswing, 100);
pos = zeros(2, length(t_sample));
for i = 1:length(t_sample)
    pos(:, i) = SwingTrajectory(t_sample(i), t0, tfswing, 0, 0.5, 0.1, 0, 0);
end

figure;
tiledlayout(2, 1);
nexttile;
plot(t_sample, pos(1, :));
xlabel("Time (s)")
ylabel("X Position")

nexttile;
plot(t_sample, pos(2, :));
xlabel("Time (s)")
ylabel("Y Position")

% Compute all the targets
controller.qtargets = zeros(robot.nj_act, length(t_sample));
controller.qtargets(:, 1) = GetActuatedCoords(q0, robot);
qfull = zeros(robot.nq, length(t_sample));
qfull(:, 1) = q0;
controller.vtargets = zeros(robot.nj_act, length(t_sample));
controller.ttargets = t_sample;
for i = 2:length(t_sample)
    pos = SwingTrajectory(t_sample(i), t0, tfswing, 0, 0.5, 0.1, 0, 0);
    FD_DELTA = 1e-8;
    pos_pert = SwingTrajectory(t_sample(i) + FD_DELTA, t0, tfswing, 0, 0.5, 0.1, 0, 0);
    vel = (pos_pert - pos)/FD_DELTA;
    
    [qfull(:, i), cost] = InverseKinematicsRobot(robot, robot.swing, robot.foot_r, qfull(:,i-1), pos);
    cost
    controller.qtargets(:,i) = GetActuatedCoords(qfull(:, i), robot);
    controller.vtargets(:,i) = GetActuatedCoords(pinv(BodyJacobianFD(robot, qfull(:,i), robot.swing, robot.foot_r))*vel, robot);
end

% Note: These are NOT used, they are merely for sim compliance
controller.q_target = zeros(2,1);
controller.v_target = zeros(2,1);

controller.p = [1000 1000 1000 1000]';
controller.d = [120 120 120 120]';
controller.saturation = [3000 3000 3000 3000];
controller.des_pos = [0.4; 0.4];

% controller.q_target = InverseKinematicsRobot(robot, robot.swing, robot.foot_r, q0, controller.des_pos);
% controller.q_target = GetActuatedCoords(controller.q_target, robot);
% controller.v_target = zeros(robot.nj_act, 1);
controller.Compute = @(t, q, v, controller) ComputeIKControl(t, q, v, controller, robot);

[t, q, v, base_pos, te, qe, ve, pos_e] = RobotSim(robot, q0, v0, pos0, t0, tf, 50, controller);

showmotion(robot, t, q);

%AnimateRobot(robot, t, q, base_pos, te, qe, ve, pos_e);

swing_pos = zeros(2, length(t));
pos_target = zeros(2, length(t));
for i = 1:length(t)
    swing_pos(:, i) = ForwardKinematics(robot, q(:, i), v(:, i), robot.swing, robot.foot_r);
    pos_target(:, i) = SwingTrajectory(t(i), t0, tfswing, 0, 0.5, 0.1, 0, 0);
end

figure;
plot(t, swing_pos(1,:));
hold on;
plot(t, swing_pos(2,:));
plot(t, pos_target(2, :));
hold off;
legend(["x", "y", "y target"]);
xlabel("Time (s)")
ylabel("Position")

figure;
hold on;
for i = 1:robot.nq
    plot(t, q(i,:))
    plot(controller.ttargets, qfull(i,:));
end
hold off;
legend(["q1", "q1 target", "q2", "q2 target", "q3", "q3 target", "q4", "q4 target", "q5", "q5 target"])
xlabel("Time (s)")
ylabel("Joint angles")

%% Helper Functions
function torques = ComputeIKControl(t, q, v, controller, robot)
    qact = GetActuatedCoords(q, robot);
    vact = GetActuatedCoords(v, robot);

    controller.q_target = interp1(controller.ttargets, controller.qtargets', t)';
    controller.v_target = interp1(controller.ttargets, controller.vtargets', t)';

    torques = PDController(t, qact, vact, controller);
end

% function qtargets = PreComputeTargets(t, q, v, controller, robot)
%     q_target = InverseKinematicsRobot(robot, robot.swing, robot.foot_r, q, controller.des_pos);
% end