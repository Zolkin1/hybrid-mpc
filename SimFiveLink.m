clc;
clear;
close all;

%% Simulate a five link robot

robot = CreateFiveLink();

q0 = zeros(robot.nq, 1);
q0(1) = 0.0;
q0(2) = 0.8;
q0(4) = 0.6;
q0(5) = 0;
v0 = zeros(robot.nv, 1);

t0 = 0;
tf = 2;

%TODO: Debug for pinned foot.

q_target = q0(3:2+robot.nj_act);
q_target(2) = q_target(2) + 0.5;
v_target = v0(3:2+robot.nj_act);

controller.q_target = q_target;
controller.v_target = v_target;
controller.p = [0 0 0 0]'; %[0 60 0 60]';
controller.d = [0 0 0 0]'; %[0 12 0 12]';
controller.saturation = [30 30 30 30];
controller.Compute = @PDController;

[t, q, qd, te, qe, qde] = RobotSim(robot, q0, v0, t0, tf, 100, controller);

dt = 0.01;

tdata = t; %sol.Time; %zeros((tf-t0)/dt, 1);
qdata = q; %zeros(robot.nq, length(tdata));
rf_pos = zeros(2, length(tdata));
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
end
[pos, vel] = ForwardKinematics(robot, q0, ...
    zeros(robot.nv, 1), 5, [0;-0.4]);
pos


plot(tdata, qdata(4, :));
hold on;
plot(tdata, q_target(2)*ones(length(qdata),1));
hold off;
xlabel("Time")
ylabel("right calf position")
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
plot(tdata, torso_pos(1, :));
hold on;
plot(tdata, torso_pos(2, :));
hold off;
xlabel("Time")
ylabel("torso pos")
legend(["x", "y"]);

showmotion(robot, tdata, qdata);