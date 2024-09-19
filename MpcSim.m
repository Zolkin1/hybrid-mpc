function [t, q, v, pos, te, qe, ve, pos_e] = MpcSim(t0, tf, q0, v0, pos0, mpc_dt, swing_params, MpcFunction, controller, robot)
%MPCSIM Simulation with MPC in the loop
%  


num_mpc_solves = ceil((tf - t0)/mpc_dt);

te = [];
qe = [];
ve = [];
pos_e = [];

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

dt_record = 0.01;

t = t0:dt_record:tf;
time_idx = 1;
q = zeros(robot.nq, length(t));
v = zeros(robot.nv, length(t));
pos = zeros(2, length(t));

figure;
tiledlayout(5, 3);

for i = 1:num_mpc_solves
    swing_params.start_pos = ForwardKinematics(robot, q0, v0, robot.swing, robot.foot_r);

    % Compute MPC
    [tmpc, umpc, qmpc, vmpc, posmpc, tempc, qempc, vempc, pos_empc, costmpc] = ...
        MpcFunction(q0, v0, pos0, swing_params, warmstart);

    % Update warmstart
    warmstart.q = qmpc;
    warmstart.v = vmpc;
    warmstart.pos = posmpc;
    warmstart.tau = umpc;

    % Update Controller
    controller.q = qmpc;
    controller.v = vmpc;
    controller.t = tmpc + t0;
    controller.u = umpc;

    % Simulate
    [sol, in_contact] = Simulate(t0, t0+min(mpc_dt, tf-t0), q0, v0, pos0, robot, controller);
    % [tsim, qsim, vsim, base_possim, tesim, qesim, vesim, pos_esim] = ...
    %     RobotSim(robot, q0, v0, pos0, t0, t0 + mpc_dt, tdensity, controller);

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
    end

    if in_contact
        % Apply the reset map
        ye = GroundContact(sol.x(end), sol.y(end));

        te = [te; sol.x(end)];
        qe = [qe, ye(1:robot.nq)];
        ve = [ve, ye(robot.nq + 1:robot.nq + robot.nv)];
        pos_e = [pos, ye(robot.nq + robot.nv + 1:end)];

        q0 = qe(:, end);
        v0 = ve(:, end);
        pos0 = pose(:, end);

        global last_contact_time;
        last_contact_time = te(end);
    end

    % Plot
    qsim = sol.y(1:robot.nq, :);
    for i = 1:robot.nq
        nexttile(3*i - 2);
        hold on;
        plot(sol.x, qsim(i, :));
        plot(tmpc + t0, qmpc(i, :), "LineStyle","--");
        hold off;
    end

    vsim = sol.y(robot.nq + 1:robot.nq + robot.nv, :);
    for i = 1:robot.nv
        nexttile(3*i - 1);
        hold on;
        plot(sol.x, vsim(i, :));
        plot(tmpc + t0, vmpc(i, :), "LineStyle","--");
        hold off;
    end

    for i = 1:robot.nj_act
        nexttile(3*i);
        hold on;
        plot(tmpc(1:end-1) + t0, umpc(i, :), "LineStyle","--");
        hold off;
    end
    pause;


    % Update other variables
    t0 = t0 + mpc_dt;
end


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
    
    if abs(t - last_contact_time) < 1e-1
        height = 0.05;
    else
        height = pos(2);
    end
    isterminal = 1;
    direction = -1;
end

function [stop, y, robot, controller] = ContactResponse(y, robot, controller)
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

    % Update the controller
    controller.q_target = -controller.q_target;
    controller.v_target = -controller.v_target;

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



