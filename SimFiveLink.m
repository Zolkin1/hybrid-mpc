clc;
clear;
close all;

%% Simulate a five link robot

robot = CreateFiveLink();

q0 = zeros(robot.nq, 1);
q0(1) = 0.0;
q0(2) = -0.2; % 0.8
q0(4) = 0.8; % 0.6
q0(5) = -0.3;
v0 = zeros(robot.nv, 1);

t0 = 0;
tf = 5.5;

q_target = GetActuatedCoords(q0, robot); %(3:2+robot.nj_act);
v_target = GetActuatedCoords(v0, robot); %(3:2+robot.nj_act);

controller.q_target = q_target;
controller.v_target = v_target;
controller.p = [3000 3000 3000 3000]'; %[0 60 0 60]';
controller.d = [120 120 120 120]'; %[0 12 0 12]';
controller.saturation = [3000 3000 3000 3000];
controller.Compute = @(t, q, v, controller) ...
    PDController(t, GetActuatedCoords(q, robot), GetActuatedCoords(v, robot), controller);

robot.torso_pos = ForwardKinematics(robot, q0, v0, 3, [0;0]);
pos0 = robot.torso_pos;

[t, q, qd, base_pos, te, qe, qde, pos_e] = RobotSim(robot, q0, v0, pos0, t0, tf, 100, controller);

dt = 0.01;

tdata = t; %sol.Time; %zeros((tf-t0)/dt, 1);
qdata = q; %zeros(robot.nq, length(tdata));
rf_pos = zeros(2, length(tdata));
stance_pos = zeros(2, length(tdata));

torso_pos = zeros(2, length(tdata));
for i = 1:length(tdata)
    %t = (i-1)*dt;
    %tdata(i) = t;
    %sol.Solution(i, :)
    %q_qd = sol.Solution(:, i); %deval(sol, t);
    %qdata(:, i) = q_qd(1:robot.nq);
    
    [pos, vel] = ForwardKinematics(robot, qdata(:, i), zeros(robot.nv, 1), 3, [0;0]);
    torso_pos(:, i) = pos(:);

    [pos, vel] = ForwardKinematics(robot, qdata(:, i), ...
       zeros(robot.nv, 1), 5, [0;-0.4]);
    rf_pos(:, i) = pos(:);

    [pos, vel] = ForwardKinematics(robot, qdata(:, i), ...
       zeros(robot.nv, 1), 0, [0;0]);
    stance_pos(:, i) = pos(:) + base_pos(:,i);
end
[pos, vel] = ForwardKinematics(robot, q0, ...
    zeros(robot.nv, 1), 5, [0;-0.4]);
pos

AnimateRobot(robot, t, q, base_pos, te, qe, qde, pos_e);

figure;
plot(tdata, qdata(2, :));
hold on;
plot(tdata, q_target(1)*ones(length(qdata),1));
hold off;
xlabel("Time")
ylabel("q2 angle (rad)")
legend(["actual", "target"])

figure;
plot(tdata, rf_pos(1, :));
hold on;
plot(tdata, rf_pos(2, :));
hold off;
xlabel("Time")
ylabel("right foot pos")
legend(["x", "y"]);

figure;
plot(tdata, stance_pos(1, :));
hold on;
plot(tdata, stance_pos(2, :));
hold off;
xlabel("Time")
ylabel("Stance foot pos")
legend(["x", "y"]);

figure;
plot(tdata, torso_pos(1, :));
hold on;
plot(tdata, torso_pos(2, :));
hold off;
xlabel("Time")
ylabel("torso pos")
legend(["x", "y"]);

figure;
plot(tdata, base_pos(1, :));
hold on;
plot(tdata, base_pos(2, :));
hold off;
xlabel("Time")
ylabel("torso pos from ode")
legend(["x", "y"]);

showmotion(robot, tdata, qdata);