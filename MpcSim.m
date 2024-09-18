function [t, q, v, pos, te, qe, ve, pos_e] = MpcSim(t0, tf, q0, v0, pos0, mpc_dt, swing_params, MpcFunction, controller, robot)
%MPCSIM Simulation with MPC in the loop
%  


num_mpc_solves = ceil((tf - t0)/mpc_dt);

t = [];
q = [];
v = [];
pos = [];

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

tdensity = 100;

figure;
tiledlayout(5, 1);

for i = 1:num_mpc_solves
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
    [tsim, qsim, vsim, base_possim, tesim, qesim, vesim, pos_esim] = ...
        RobotSim(robot, q0, v0, pos0, t0, t0 + mpc_dt, tdensity, controller);

    t
    tsim
    t = [t, tsim];
    q = [q, qsim];
    v = [v, vsim];
    pos = [pos, base_possim];
     
    te = [te; tesim];
    qe = [qe, qesim];
    ve = [ve, vesim];
    pos_e = [pos, pos_esim];

    % Plot
    for i = 1:robot.nq
        nexttile(i);
        hold on;
        plot(tsim, qsim(i, :));
        plot(tmpc + t0, qmpc(i, :), "LineStyle","--");
        hold off;
    end
    pause;


    % Update other variables
    t0 = t0 + mpc_dt;

    q0 = q(:, end);
    v0 = v(:, end);
    pos0 = pos(:, end);
end


end

