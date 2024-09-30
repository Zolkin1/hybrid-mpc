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

%% Ground description
ground.xlength = [.4, .5, .5, 1.5];
ground.start_height = [0, 0.075, 0.125, 0.15];
ground.end_height = [0, 0.075 0.125, 0.15];

%% Long Horizon MPC
swing_params.apex = 0.1;
swing_params.num_swings = 2; %6;
swing_params.nodes = 25*ones(swing_params.num_swings, 1);
swing_params.start_pos = ForwardKinematics(robot, q0, v0, robot.swing, robot.foot_r);
swing_params.time_into_swing = 0.0;
swing_params.no_swing_constraint = swing_params.num_swings + 1;
swing_params.ground_idx = [1 2 2 3 3 4 4]; %ones(swing_params.num_swings, 1);
swing_params.tf = 0.3;

cost_params.pose_target = [0.; 1.5; 0.2; 0.8; -0.4];
cost_params.pose_weight = [8000, 5, 5, 5, 5]';
%cost_params.pose_weight = [1000; 1000; 1000; 1000; 1000];

cost_params.vel_target = [0, 0, 0, 0, 0]';
cost_params.vel_weight = 0*[5., 25., 25., 25., 5.]';

cost_params.pos_target = [2, 0.8]';
cost_params.pos_weight = [3000, 1000]';

cost_params.u_target = [0, 0, 0, 0, 0]';
cost_params.u_weight = [10, 10, 10, 10, 10]';

cost_params.swing_params = swing_params;
cost_params.swing_weights = 0*[1, 1]';

% cost_params.head_target = [0; 1.6];
% cost_params.head_weight = [0; 7000];
cost_params.head_target = [0.2; 1.6];
cost_params.head_weight = [10000; 700];

costfcn = @(x, u, t, dt) ...
    ComputeStageCost(x, u, t, dt, robot, cost_params);

total_nodes = sum(swing_params.nodes);
warmstart.q = zeros(robot.nq, total_nodes);
warmstart.v = zeros(robot.nv, total_nodes);
warmstart.pos = zeros(2, total_nodes);
warmstart.u = zeros(robot.nj_act, total_nodes);

for node = 1:total_nodes
    warmstart.q(:, node) = q0;
    warmstart.v(:, node) = v0;
    warmstart.pos(:, node) = pos0;
    warmstart.u(:, node) = zeros(robot.nj_act, 1);
end

% The idea here will be whenever the x position is within some distance of
% new terrain then solve with the first step fixed on both pieces of
% terrain, and choose the one with lower cost.
% Another option is to use raibert in the mpc to determine where the foot should be 
% then use that to fix the terrain sequence. Would probably iterate through
% raibert with fixed mode MPC as the hybrid MPC. Like use raibert based on
% the initial state of the swing, then fix the terrain and solve, then do
% this again. (?) Is this really what I want - probably simple and will
% work, so lets do that.

mpc_sol = MultiStepMPCTerrain(robot, q0, v0, pos0, swing_params, ground, costfcn, warmstart);

mpc_plot = figure;
PlotMPCSol(robot, ground, mpc_sol, mpc_plot);

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
controller = CreateController(mpc_sol);

% ode45 until a contact event occurs
options = odeset('Events', @(t,y)GroundContact(t,y,robot,ground),'Refine',8);
global last_contact_time;
y0 = [q0; v0; ForwardKinematics(robot, q0, v0, 3, [0;0])];
tf = 1; %1.5; %2.5; %t(end); % 3

tesim = [];
qesim = [];
vesim = [];
pos_esim = [];

tsim = [];
qsim = [];
possim = [];
mpc_dt = 0.1;
tic
while t0 < tf
    sol = ode45(@(t, y) RobotDynamics(t, y, robot, controller), [t0, min(tf, t0 + mpc_dt)], y0, options);

    if (~isempty(sol.xe))
        disp("Contact")
        t0 = sol.xe(1);

        ye = deval(sol, sol.xe(1));

        [stop, y0, robot, controller] = ContactResponse(sol.xe(1), ye, robot, controller);

        tesim(:, end + 1) = t0;
        qesim(:, end + 1) = y0(1:robot.nq, end);
        vesim(:, end + 1) = y0(robot.nq + 1:robot.nq + robot.nv);
        pos_esim(:, end + 1) = y0(11:12, end);

        last_contact_time = t0;
        %break;
    
        i = 1;
        thist = sol.x(i);
        while i <= length(sol.x) && thist < t0
            tsim = [tsim, sol.x(i)];
            thist = tsim(end);
            qsim = [qsim, sol.y(1:robot.nq, i)];
            possim = [possim, sol.y(11:12, i)];
            i = i + 1;
        end
        
        % I think the position gets weird when I'm on a step
        q0 = y0(1:robot.nq);
        v0 = y0(robot.nq + 1:robot.nq + robot.nv);
        pos0 = y0(11:12);
        swing_params.ground_idx(1) = [];
        swing_params.ground_idx(end + 1) = swing_params.ground_idx(end);
        swing_params.time_into_swing = 0;
        % TODO: Warmstart the MPC
        mpc_sol = MultiStepMPCTerrain(robot, q0, v0, pos0, swing_params, ground, costfcn, warmstart);
        PlotMPCSol(robot, ground, mpc_sol, mpc_plot);

        mpc_sol.t = mpc_sol.t + t0;
        mpc_sol.te = mpc_sol.te + t0;

        controller = CreateController(mpc_sol);

        % controller.q = mpc_sol.q;
        % controller.v = mpc_sol.v;
        % controller.t = mpc_sol.t;
        % controller.u = mpc_sol.u;
        % controller.Compute = @(tc, qc, vc, controller) MpcController(tc, qc, vc, controller);
    else
        y0 = sol.y(:, end);
        t0 = sol.x(end);

        if sol.x(end) < tf
            % Plan MPC without adjusting the ground contacts
            q0 = y0(1:robot.nq);
            v0 = y0(robot.nq + 1:robot.nq + robot.nv);
            pos0 = y0(11:12);
            if ~isempty(tesim)
                swing_params.time_into_swing = sol.x(end) - tesim(end);
            else
                swing_params.time_into_swing = sol.x(end);
            end
            % TODO: Warmstart the MPC
            mpc_sol = MultiStepMPCTerrain(robot, q0, v0, pos0, swing_params, ground, costfcn, warmstart);

            mpc_sol.t = mpc_sol.t + t0;
            mpc_sol.te = mpc_sol.te + t0;

            controller = CreateController(mpc_sol);
            PlotMPCSol(robot, ground, mpc_sol, mpc_plot);
        end

        tsim = [tsim, sol.x];
        qsim = [qsim, sol.y(1:robot.nq, :)];
        possim = [possim, sol.y(11:12, :)];
    end

    tsim(end)
    tesim
end
toc

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

% MPC
AnimateRobotGround(robot, t, q, pos, te, qe, ve, pos_e, ground);

% Simulation
AnimateRobotGround(robot, tsim, qsim, possim, tesim, qesim, vesim , pos_esim, ground);
%% Controllers
function tau = MpcController(tc, qc, vc, controller)
    if tc > controller.t(end)
        uff = controller.u(:, end);
        controller.q_target = controller.q(:, end);
        controller.v_target = controller.v(:, end);
    
        controller.feed_forward = 0*uff;
    else
        uff = interp1(controller.t(1:end), controller.u', tc)';
        controller.q_target = interp1(controller.t, controller.q', tc)';
        controller.v_target = interp1(controller.t, controller.v', tc)';
    
        controller.feed_forward = uff; % zeros(length(uff), 1);
    end

    tau = PDController(tc, qc, vc, controller);
end

function controller = CreateController(mpc_sol)
    controller.p = 1*[7000; 7000; 7000; 7000; 7000];
    controller.d = 1*[100; 100; 100; 100; 100];
    controller.saturation = [30000 30000 30000 30000 30000];
    
    % Drive to the next impact only
    te = mpc_sol.te(1);
    i = 0;
    while mpc_sol.t(i + 1) <= mpc_sol.te(1)
        i = i + 1;
    end
    i
    controller.q = mpc_sol.q(:, 1:i);
    controller.v = mpc_sol.v(:, 1:i);
    controller.t = mpc_sol.t(1:i);
    controller.u = mpc_sol.u(:, 1:i);
    controller.Compute = @(tc, qc, vc, controller) MpcController(tc, qc, vc, controller);
end

%% Contacts
function [height, isterminal, direction] = GroundContact(t, y, robot, ground)
    global last_contact_time;
    %persistent last_contact_time;   % To store the time of the last event
    if isempty(last_contact_time)
        last_contact_time = -inf;
    end
    % Return distance of foot from ground
    q = y(1:robot.nq);
    v = y(robot.nq + 1: robot.nq + robot.nv);
    [pos, vel] = ForwardKinematics(robot, q, v, robot.swing, [0;-robot.calf_length]);
    
    % TODO: Add the offset to get position in the global frame
    fk = ForwardKinematics(robot, q, v, 3, [0;0]);
    position_diff = y(11:end) - fk;
    swing_xpos_global = position_diff(1) + pos(1);
    swing_ypos_global = position_diff(2) + pos(2);
    
    gheight = GroundHeight(swing_xpos_global, ground);

    if abs(t - last_contact_time) < 1e-2
       height = 0.05;
    else
        height = swing_ypos_global - gheight;
    end
    isterminal = 1;
    direction = -1;
end

function height = GroundHeight(x, ground)
    if x <= 0
        height = 0;
        return;
    end

    gx = 0;
    i = 1;
    while gx < x
        if i > length(ground.xlength)
            x = 0;
            height = 0;
            return;
            warning("No ground defined here!");
        end
        gx = gx + ground.xlength(i);
        i = i + 1;
    end

    % For now use only the end height
    height = ground.end_height(i - 1);
end

function dydt = RobotDynamics(t, y, robot, controller)
    dydt = zeros(2*robot.nv + 2, 1);
    q = y(1:robot.nq);
    v = y(robot.nq + 1: robot.nq + robot.nv);
    dydt(1:robot.nv) = v;

    %q_act = GetActuatedCoords(q, robot);
    %v_act = GetActuatedCoords(v, robot);
    tau_act = controller.Compute(t, q, v, controller);

    dydt(robot.nv + 1: 2*robot.nv) = ...
        FDab(robot, q, v, tau_act);

    J = BodyJacobianFD(robot, q, 3, [0;0]);
    dydt(2*robot.nv+1:end) = J*v;
end

function [stop, y, robot, controller] = ContactResponse(t, y, robot, controller)
    q = y(1:robot.nq);
    v = y(robot.nq+1:robot.nq+robot.nv);

    % Reset map is given by
    % q+ = (I - M^-1*J^T*(J*M^-1*J^T)^-1*J)q-
    J = BodyJacobianFD(robot, q, robot.swing, robot.foot_r);
    [M, C] = HandC(robot, q, v, {});
    Minv = inv(M);
    vplus = (eye(robot.nq) - Minv*J'*inv(J*Minv*J')*J)*v;

    % Now switch the joints
    % Calculate angle to the ground
    pos_joint = ForwardKinematics(robot, q, v, robot.swing, [0;0]);
    pos_foot = ForwardKinematics(robot, q, v, robot.swing, robot.foot_r);
    pos = pos_joint - pos_foot;
    q(1) = -atan2(pos(1), pos(2));
    q(2) = -q(5);
    q(3) = -q(4);

    q(4) = -y(3);
    q(5) = -y(2);

    % Remap velocities
    vplus_temp = vplus;
    % TODO: What is the rotational velocity of the new ground contact?
    %vplus(1) = -vplus(1); %0.5;
    vplus(2) = -vplus(5);
    vplus(3) = -vplus(4);
    vplus(4) = -vplus_temp(3);
    vplus(5) = -vplus_temp(2);

    % Assign to y
    y(1:robot.nq) = q;
    y(robot.nq+1:robot.nq+robot.nv) = vplus;

    stop = false;
end

%% Plotting
function PlotMPCSol(robot, ground, mpc_sol, mpc_plot)
    figure(mpc_plot);
    % Plot the IC of the five link
    q = mpc_sol.q(:, 1);
    qd = mpc_sol.v(:, 1);
    pos = mpc_sol.pos(:, 1);

    torso_fk = ForwardKinematics(robot, q, qd, 3, [0;0]);
    offset = pos - torso_fk;

    % Compute FK to get all the ends of the links
    joint_positions = zeros(2, robot.nq);
    link_positions = zeros(2, robot.nq);
    for j = 1:robot.nq
        joint_positions(:,j) = ForwardKinematics(robot, q, qd, j, [0;0]);
        link_positions(:, j) = ForwardKinematics(robot, q, qd, j, robot.link{j});
    end

    hold on;
    axis equal;
    scatter(pos(1), pos(2), [], [0.4940 0.1840 0.5560], "filled");
    
    for j = 1:robot.nq
        joint_positions(:,j) = joint_positions(:,j) + offset;
        link_positions(:, j) = link_positions(:, j) + offset;
    end
    for j = 1:robot.nq
        plot([joint_positions(1,j), link_positions(1,j)], [joint_positions(2,j), link_positions(2,j)], ...
            "Color", "#0072BD", "LineWidth", 2);
        scatter(joint_positions(1,j), joint_positions(2,j), [], [0.6350 0.0780 0.1840], "filled");
    end
    
    PlotGround(ground);

    % Plot swing traj and torso traj
    % swing_pos = [];
    % torso_pos = [];
    % for i = 1:length(mpc_sol.q)
    %     swing_pos(:, end + 1) = ForwardKinematics(robot, mpc_sol.q(:, i), qd, robot.swing, robot.foot_r);
    %     torso_pos(:, end + 1) = ForwardKinematics(robot, mpc_sol.q(:, i), qd, 3, [0; 0]);
    % end
    % 
    % plot(swing_pos(1, :), swing_pos(2, :));
    % plot(torso_pos(1, :), torso_pos(2, :));

    hold off;
end

function PlotGround(ground)
% always starts at 0, plot a little before it
plot([-0.5, 0], ...
        [ground.start_height(1),...
         ground.end_height(1)], "LineWidth", 2, "Color", "k");
x0 = 0;
for i = 1:length(ground.xlength)
    plot([x0, x0 + ground.xlength(i)], ...
        [ground.start_height(i),...
         ground.end_height(i)], "LineWidth", 2, "Color", "k");
    x0 = x0 + ground.xlength(i);
end
end
