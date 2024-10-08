function sim_sol = MpcSim(t0, tf, q0, v0, pos0, mpc_dt, short_mpc, long_mpc, controller, robot)
%MPCSIM Simulation with MPC in the loop
%  


%num_mpc_solves = ceil((tf - t0)/mpc_dt);

te = [];
qe = [];
ve = [];
pos_e = [];

short_warmstart = MakeBasicWarmstart(q0, v0, pos0, sum(short_mpc.swing_params.nodes), robot);
long_warmstart = MakeBasicWarmstart(q0, v0, pos0, sum(long_mpc.swing_params.nodes), robot);

dt_record = 0.01;

t = t0:dt_record:tf;
time_idx = 1;
q = zeros(robot.nq, length(t));
v = zeros(robot.nv, length(t));
pos = zeros(2, length(t));

tracking = figure;
tiledlayout(5, 3);

impacts = figure;
tiledlayout(5, 3);

num_contacts = 0;

short_mpc.swing_params.no_swing_constraint = 2;

% Initial high level MPC solve
long_mpc_sol = long_mpc.mpc(q0, v0, pos0, long_mpc.swing_params, long_warmstart, long_mpc.cost_fcn);
AnimateRobot(robot, long_mpc_sol.t, long_mpc_sol.q, long_mpc_sol.pos, long_mpc_sol.te, long_mpc_sol.qe, long_mpc_sol.ve, long_mpc_sol.pos_e);

short_mpc.swing_params.terminal_constraint = UpdateTerminalConstraint(long_mpc_sol, 1);
short_mpc.swing_params.terminal_constraint

short_mpc.cost_params = UpdateCostParams(long_mpc_sol, num_contacts + 1, robot);
% short_mpc.cost_params.pose_target = short_mpc.swing_params.terminal_constraint(1:robot.nq);
% short_mpc.cost_params.vel_target = short_mpc.swing_params.terminal_constraint(robot.nq + 1:robot.nq + robot.nv);
% short_mpc.cost_params.pos_target = short_mpc.swing_params.terminal_constraint(11:end);
short_mpc.cost_fcn = @(x, u, t, dt, swing_num) ...
    SinglePhaseTrackingCost(t, x, u, dt, robot, short_mpc.cost_params);
%short_mpc.swing_params.length(1) = long_mpc_sol.swing_xpos(1);
%short_mpc.swing_params.tf(1) = long_mpc_sol.swing_tf(1);
%mpc_dt = short_mpc.swing_params.tf(1);

short_warmstart = UpdateWarmStart(short_mpc, robot);

% Plots for debugging     
debugging = figure;
tiledlayout(5,3);
for i = 1:robot.nq
    nexttile(i*3 - 2);
    hold on;
    plot(short_mpc.cost_params.t_ref, short_mpc.cost_params.q_target(i, :));
    hold off;

    hold on;
    nexttile(i*3 - 1);
    plot(short_mpc.cost_params.t_ref, short_mpc.cost_params.v_target(i, :));
    hold off;

    hold on;
    nexttile(i*3);
    plot(short_mpc.cost_params.t_ref, short_mpc.cost_params.u_target(i, :));
    hold off;
end

while t0 < tf
    short_mpc.swing_params.start_pos = ForwardKinematics(robot, q0, v0, robot.swing, robot.foot_r);

    % Compute MPC
    [tmpc, umpc, qmpc, vmpc, posmpc, tempc, qempc, vempc, pos_empc, costmpc] = ...
        short_mpc.mpc(q0, v0, pos0, short_mpc.swing_params, short_warmstart, short_mpc.cost_fcn);
    AnimateRobot(robot, tmpc, qmpc, posmpc, tempc, qempc, vempc, pos_empc);

    % Plots for debugging
    figure(debugging);
    clf;
    for i = 1:robot.nq
        nexttile(i*3 - 2);
        hold on;
        plot(tmpc, qmpc(i, :));
        hold off;

        nexttile(i*3 - 1);
        hold on;
        plot(tmpc, vmpc(i, :));
        hold off;

        nexttile(i*3);
        hold on;
        plot(tmpc(1:end-1), umpc(i, :));
        hold off;
    end

    figure(debugging);
    for i = 1:robot.nq
        nexttile(i*3 - 2);
        hold on;
        plot(tmpc(1:end-1), short_warmstart.q(i, :));
        hold off;

        nexttile(i*3 - 1);
        hold on;
        plot(tmpc(1:end-1), short_warmstart.v(i, :));
        hold off;

        nexttile(i*3);
        hold on;
        plot(tmpc(1:end-1), short_warmstart.tau(i, :));
        hold off;
    end

    % Update Controller
    controller.q = qmpc;
    controller.v = vmpc;
    controller.t = tmpc + t0;
    controller.u = umpc;

    % Simulate
    [sol, in_contact] = Simulate(t0, t0+min(mpc_dt, tf-t0), q0, v0, pos0, robot, controller);

    in_contact
    sol.x(end)

    % Record all the data into the associated structs
    while time_idx <= length(t) && t(time_idx) <= sol.x(end)
        y = deval(sol, t(time_idx));
        q(:, time_idx) = y(1:robot.nq);
        v(:, time_idx) = y(robot.nq + 1:robot.nq + robot.nv);
        pos(:, time_idx) = y(robot.nq + robot.nv + 1:end);
        time_idx = time_idx + 1;

        q0 = q(:, time_idx - 1);
        v0 = v(:, time_idx - 1);
        pos0 = pos(:, time_idx - 1);

        short_mpc.swing_params.start_pos = ForwardKinematics(robot, q(:,end), v(:,end), robot.swing, robot.foot_r);
    end

    if in_contact
        % Apply the reset map
        [stop, ye] = ContactResponse(sol.y(:, end), robot);

        te = [te; sol.x(end)];
        qe = [qe, ye(1:robot.nq)];
        ve = [ve, ye(robot.nq + 1:robot.nq + robot.nv)];
        pos_e = [pos, ye(robot.nq + robot.nv + 1:end)];

        q0 = qe(:, end);
        v0 = ve(:, end);
        pos0 = pos_e(:, end);

        % Plot the impact states
        figure(impacts);
        for i = 1:robot.nq
            nexttile(3*i - 2);
            hold on;
            scatter(num_contacts, qe(i, end), "blue", "filled");
            scatter(num_contacts, long_mpc_sol.qe(i, 1));
            hold off;

            nexttile(3*i - 1);
            hold on;
            scatter(num_contacts, ve(i, end), "blue", "filled");
            scatter(num_contacts, long_mpc_sol.ve(i, 1));
            hold off;
        end

        global last_contact_time;
        last_contact_time = te(end);

        % Replan high level MPC solve
        long_mpc_sol = long_mpc.mpc(q0, v0, pos0, long_mpc.swing_params, long_warmstart, long_mpc.cost_fcn);
        
        short_mpc.swing_params.terminal_constraint = UpdateTerminalConstraint(long_mpc_sol, 1);
        short_mpc.swing_params.terminal_constraint

        short_mpc.cost_params = UpdateCostParams(long_mpc_sol, 1, robot);
        % short_mpc.cost_params.pose_target = short_mpc.swing_params.terminal_constraint(1:robot.nq);
        % short_mpc.cost_params.vel_target = short_mpc.swing_params.terminal_constraint(robot.nq + 1:robot.nq + robot.nv);
        % short_mpc.cost_params.pos_target = short_mpc.swing_params.terminal_constraint(11:end);
        short_mpc.cost_fcn = @(x, u, t, dt, swing_num) ...
            SinglePhaseTrackingCost(t, x, u, dt, robot, short_mpc.cost_params);
        %short_mpc.swing_params.length(1) = long_mpc_sol.swing_xpos(num_contacts + 2);
        %short_mpc.swing_params.tf(1) = long_mpc_sol.swing_tf(num_contacts + 2);
        mpc_dt = short_mpc.swing_params.tf(1);

        num_contacts = num_contacts + 1;
        % Adjust short term MPC targets
        if num_contacts >= 3
            short_mpc.swing_params.no_swing_constraint = 1;
        end
        short_mpc.swing_params.start_pos = ForwardKinematics(robot, qe(:,end), ve(:,end), robot.swing, robot.foot_r);

        short_warmstart = UpdateWarmStart(short_mpc, robot);
    else
        short_mpc.cost_params.t_ref = short_mpc.cost_params.t_ref - sol.x(end);
        short_mpc.cost_fcn = @(x, u, t, dt, swing_num) ...
            SinglePhaseTrackingCost(t, x, u, dt, robot, short_mpc.cost_params);
    end

    if ~isempty(te)
        short_mpc.swing_params.time_into_swing = sol.x(end) - te(end);
    else
        short_mpc.swing_params.time_into_swing = sol.x(end);
        if short_mpc.swing_params.time_into_swing >= short_mpc.swing_params.tf(1)
            short_mpc.swing_params.time_into_swing = short_mpc.swing_params.tf(1) - 0.02;
        end
    end


    % Plot
    t_offset = t0;
    if length(te) > 1
        %t_offset = te(end - 1) + sol.x(end);
    end

    figure(tracking);
    qsim = sol.y(1:robot.nq, :);
    for i = 1:robot.nq
        nexttile(3*i - 2);
        hold on;
        plot(sol.x, qsim(i, :), 'LineWidth', 2, "Color", [0 0.4470 0.7410]);
        plot(tmpc + t_offset, qmpc(i, :), "LineStyle", "--",...
            'LineWidth', 2, "Color", [0.8500 0.3250 0.0980]);
        scatter(sol.x(1), qsim(i, 1), "magenta", "filled");
        hold off;
    end

    vsim = sol.y(robot.nq + 1:robot.nq + robot.nv, :);
    for i = 1:robot.nv
        nexttile(3*i - 1);
        hold on;
        plot(sol.x, vsim(i, :), 'LineWidth', 2, "Color", [0 0.4470 0.7410]);
        plot(tmpc + t_offset, vmpc(i, :), "LineStyle", "--",...
            'LineWidth', 2, "Color", [0.8500 0.3250 0.0980]);
        scatter(sol.x(1), vsim(i, 1), "magenta", "filled");
        hold off;
    end

    for i = 1:robot.nj_act
        nexttile(3*i);
        hold on;
        plot(tmpc(1:end-1) + t_offset, umpc(i, :), "LineStyle", "--",...
            'LineWidth',2, "Color", [0.8500 0.3250 0.0980]);
        scatter(tmpc(1) + t_offset, umpc(i, 1), "magenta", "filled");
        hold off;
    end
    %pause;


    % Update other variables
    t0 = sol.x(end);
end

sim_sol.q = q;
sim_sol.v = v;
sim_sol.pos = pos;
sim_sol.t = t;
sim_sol.qe = qe;
sim_sol.ve = ve;
sim_sol.pos_e = pos_e;
sim_sol.te = te;


end

function [sol, in_contact] = Simulate(t0, tf, q0, v0, pos0, robot, controller)
    y0 = [q0; v0; pos0];
    options = odeset('Events', @(t,y)GroundContact(t,y,robot),'Refine',8);
    sol = ode45(@(t, y) odefun(t, y, robot, controller), [t0, tf], y0, options);
    in_contact = false;
    if (sol.x(end) < tf)
        in_contact = true;
    end
end

function [height, isterminal, direction] = GroundContact(t, y, robot)
    global last_contact_time;
    %persistent last_contact_time;   % To store the time of the last event
    if isempty(last_contact_time)
        last_contact_time = -inf;
    end
    % Return distance of foot from ground
    q = y(1:robot.nq);
    v = y(robot.nq + 1: robot.nq + robot.nv);
    [pos, vel] = ForwardKinematics(robot, q, v, robot.swing, [0;-robot.calf_length]);
    
    if abs(t - last_contact_time) < 1e-2
       height = 0.05;
    else
        height = pos(2);
    end
    isterminal = 1;
    direction = -1;
end

function [stop, y] = ContactResponse(y, robot)
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

function dydt = odefun(t, y, robot, controller)
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
    % TODO: Do torso acc via casadi
end

function warmstart = MakeBasicWarmstart(q0, v0, pos0, total_nodes, robot)
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
end

function terminal_constraint = UpdateTerminalConstraint(long_mpc_sol, contact_num)
    % Get the index
    i = length(long_mpc_sol.t);
    if ~isempty(long_mpc_sol.te)
        i = 1;
        while i <= length(long_mpc_sol.t) && long_mpc_sol.t(i) < long_mpc_sol.te(contact_num)
            i = i + 1;
        end
    
        i = i - 1;
    end

    terminal_constraint = [long_mpc_sol.q(:, i); long_mpc_sol.v(:, i); long_mpc_sol.pos(:, i)];
end

function cost_params = UpdateCostParams(long_mpc_sol, contact_num, robot)
    % Get the index
    i = 1;
    while long_mpc_sol.t(i) < long_mpc_sol.te(contact_num)
        i = i + 1;
    end

    i = i - 1;
    cost_params.q_target = zeros(robot.nq, i);
    cost_params.v_target = zeros(robot.nv, i);
    cost_params.u_target = zeros(robot.nj_act, i);
    cost_params.pos_target = zeros(2, i);

    for k = 1:i
        cost_params.t_ref(k) = long_mpc_sol.t(k);
        cost_params.q_target(:, k) = long_mpc_sol.q(:, k);
        cost_params.v_target(:, k) = long_mpc_sol.v(:, k);
        cost_params.u_target(:, k) = long_mpc_sol.u(:, k);
        cost_params.pos_target(:, k) = long_mpc_sol.pos(:, k);
    end

    cost_params.q_weight = 40*[500, 500, 500, 500, 500]';
    cost_params.v_weight = [500., 500., 500., 500., 500.]';
    cost_params.u_weight = [10, 10, 10, 10, 10]';
    cost_params.pos_weight = [300, 1000]';

end

function warmstart = UpdateWarmStart(mpc, robot)
    total_nodes = sum(mpc.swing_params.nodes);

    warmstart.q = zeros(robot.nq, total_nodes);
    warmstart.v = zeros(robot.nv, total_nodes);
    warmstart.pos = zeros(2, total_nodes);
    warmstart.tau = zeros(robot.nj_act, total_nodes);
    
    t = linspace(0, mpc.swing_params.tf, total_nodes);

    for node = 1:total_nodes
        for i = 1:robot.nq
            warmstart.q(i, node) = interp1(mpc.cost_params.t_ref, mpc.cost_params.q_target(i, :), t(node), "linear", mpc.cost_params.q_target(i, end));
            warmstart.v(i, node) = interp1(mpc.cost_params.t_ref, mpc.cost_params.v_target(i, :), t(node), "linear", mpc.cost_params.v_target(i, end));
            warmstart.tau(i, node) = interp1(mpc.cost_params.t_ref, mpc.cost_params.u_target(i, :), t(node), "linear", mpc.cost_params.u_target(i, end));
        end

        for i = 1:2
            warmstart.pos(:, node) = interp1(mpc.cost_params.t_ref, mpc.cost_params.pos_target(i, :), t(node), "linear", mpc.cost_params.pos_target(i, end));
        end
    end
end

