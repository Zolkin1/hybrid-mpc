clc;
clear;
close all;

% TODO:
% - 

%%
BallParams.M = 1;
BallParams.r = 0.1;
BallParams.g = [0; -9.81];
BallParams.gamma = [1; 1]; % 0.8
BallParams.drag = -0.02;

BallParams.ForceLim = [100; 100]; % [200; 200]

pos0 = [1, 1];


GuardParams.num = 3;
GuardParams.G{1} = [0, 0; 8, 0]; % Wall on the x axis
GuardParams.G{2} = [0, 0; 0, 8]; % Wall on the y axis
GuardParams.G{3} = [3, 3, 1]; % x, y, radius

GuardParams.Color(1) = "k";
GuardParams.Color(2) = "k";
GuardParams.Color(3) = "b";

GuardParams.Direction{1} = [-1, 2, 0, 8]; % inc.dec value, x/y constraint, start value, end value
GuardParams.Direction{2} = [-1, 1, 0, 8];
GuardParams.Direction{3} = [0, 1, 2, 4];

GuardParams.Value(1) = 0;
GuardParams.Value(2) = 0;
GuardParams.Value(3) = 2;

%% Cost Parameters
CostParams.position_target = [3; 3];
CostParams.position_weight = [50; 50];
CostParams.velocity_target = [0; 0];
CostParams.velocity_weight = 0.001*[1; 1];
CostParams.u_target = [0; 0];
CostParams.u_weight = 0.001*[1; 1];

costfcn = @(x, u, dt, mode_type) BallStageCost(x, u, dt, mode_type, CostParams);

%%
t0 = 0;
tf = 3; %5;

controller.Compute = @(t, x) ZeroController();

mpc_dt = 0.01;

%x0_og = [3; 6; 0; 0];
x0_og = [1; 1; 0; 0];
x0 = x0_og;

%% Control loop simulation
% Simulate
tsim = t0;
mpc_sol = [];
x0 = x0_og;

% Compute CEM
cem_controller_start = CreateCEMControllerBall(tf - t0, x0, BallParams, GuardParams, CostParams);

% Simulate CEM
tsim = t0;
cem_sol = [];
x0 = x0_og;
while tsim < tf
    cem_sol = [cem_sol, BallSimulation(tsim, tf, x0, cem_controller_start, BallParams, GuardParams)];
    x0 = ResetMap(cem_sol(end), BallParams, GuardParams);
    tsim = cem_sol(end).x(end);
end

%AnimateBall(cem_sol, BallParams, GuardParams);

x0 = x0_og;
domains_start = ExtractDomainInfo(x0, (tf-t0), cem_controller_start, BallParams, GuardParams);
domains_start
terminal = false;
for dom = 1:length(domains_start.guard_idx)
    if domains_start.guard_idx(dom) == 3
        terminal = true;
    end
end

if ~terminal
    error("CEM did not find the terminal constraint");
end

warmstart = CreateWarmstart(cem_sol, cem_controller_start, domains_start);


rng("default");
num_sims = 10; %10;
for i = 1:num_sims
    % Generate all the disturbance sequences
    dist_dt = 0.05;
    %w_mag = [5; 5];
    w_mag = [20; 20];
    tw = t0:dist_dt:tf;
    w_seq{i} = zeros(2, length(tw));
    w_seq{i}(1,:) = 2*w_mag(1)*(rand(1, length(tw)) - 0.5);
    w_seq{i}(2,:) = 2*w_mag(2)*(rand(1, length(tw)) - 0.5);
    w{i} = @(t) interp1(tw, w_seq{i}', t, "previous")';
    %w = @(t) zeros(2,1);
end

if num_sims == 0
    w_mag = [0; 0];
end
w{num_sims + 1} = @(t) zeros(2,1);

paths = figure;
sucesses = 0;
tic
for i = 1:num_sims + 1
    tsim = t0;
    mpc_sol = [];
    x0 = x0_og;
    %mpc_sim = figure();
    %tiledlayout(2, 3);
    domains = domains_start;
    warmstart = CreateWarmstart(cem_sol, cem_controller_start, domains_start);
    w_opt = [];
    while tsim < tf
        % TODO: Update warm start
    
        % Compute MPC
        w_opt = [];
        [tmpc, xmpc, umpc, te, xe, domT, w_opt] = BouncingBallMPC(domains, costfcn, x0, warmstart, BallParams, GuardParams, w_opt);
        
        % for d = 1:domains.num
        %     domains.T(d) = domT(d);
        % end

        % Create MPC controller
        mpc_controller.Compute = @(t, x) MpcController(t, x, umpc, tmpc + tsim);
    
        % Simulate
        %mpc_sol = [mpc_sol, BallSimulation(tsim, min(tf, tsim + mpc_dt), x0, mpc_controller, BallParams, GuardParams)];
        mpc_sol = [mpc_sol, BallSimulationDisturbance(tsim, ...
            min(tf, tsim + mpc_dt), x0, mpc_controller, BallParams,...
            GuardParams, w{i})];
    
        %figure(mpc_sim);
        %PlotSimAndTraj(mpc_sol(end), tmpc, xmpc, umpc);
    
        x0 = mpc_sol(end).y(:, end);
        if ~isempty(mpc_sol(end).ie) && mpc_sol(end).ie(1) ~= 3
            x0 = ResetMap(mpc_sol(end), BallParams, GuardParams);
    
            % Compute CEM
            cem_controller_new = CreateCEMControllerBall(tf - t0, x0, BallParams, GuardParams, CostParams);

            domains_new = ExtractDomainInfo(x0, (tf-t0), cem_controller_new, BallParams, GuardParams);
            domains_new

            terminal = false;
            for dom = 1:length(domains_new.guard_idx)
                if domains_new.guard_idx(dom) == 3
                    terminal = true;
                end
            end

            if ~terminal
                warning("CEM did not find the terminal constraint");
            else
                domains = domains_new;
                warmstart = CreateWarmstart(cem_sol, cem_controller_new, domains_new);
            end
        else
            nodes_per_sec = 60;
            elapsed_time = mpc_sol(end).x(end) - mpc_sol(end).x(1);
            if domains.T(1) > elapsed_time
                domains.T(1) = domains.T(1) - elapsed_time;
                domains.nodes(1) = domains.nodes(1) - floor(nodes_per_sec*elapsed_time);
                domains.nodes(end) = domains.nodes(end) + floor(nodes_per_sec*elapsed_time);

                if domains.nodes(1) <= 0
                    error("Negative or no node domain!");
                end

                % if sum(domains.T) ~= tf
                %     error("Domains lost or gained time");
                % end
            else
                diff = 0;
                t_count = domains.T(1);
                removed_time = 0;
                while t_count < elapsed_time
                    removed_time = removed_time + domains.T(1);
                    diff = elapsed_time - domains.T(1);
                    domains.T(1) = [];
                    t_count = t_count + domains.T(1);
                    domains.type(1) = [];
                    domains.guard_idx(1) = [];
                    if length(domains.variable) > 1
                        domains.variable(1) = [];
                        domains.guard_constraint(1) = [];
                        domains.guard_val(1) = [];
                    end
                    node_count = domains.nodes(1);
                    domains.nodes(1) = [];
                    domains.num = domains.num - 1;
                end
                
                node_count = node_count + floor(diff*nodes_per_sec);
                domains.T(1) = domains.T(1) - (elapsed_time - removed_time);
                domains.nodes(1) = domains.nodes(1) - floor(diff*nodes_per_sec);
                domains.nodes(end) = domains.nodes(end) + node_count;

                if domains.nodes(1) <= 0
                    error("Negative or no node domain!");
                end

                % if sum(domains.T) ~= tf
                %     error("Domains lost or gained time");
                % end
            end
            domains.T(end) = domains.T(end) + elapsed_time;

            if abs(sum(domains.T) - tf) > 1e-4
                error("Domains lost or gained time");
            end

            if sum(domains.nodes) ~= floor(sum(domains.nodes))
                error("Nodes non integer!");
            end

        end
        tsim = mpc_sol(end).x(end);
    end
    
    tsim_plot = linspace(mpc_sol(1).x(1), mpc_sol(end).x(end), 400);
    sim_pos = zeros(2, length(tsim_plot));
    for j = 1:length(tsim_plot)
        x_plot = GetState(mpc_sol, tsim_plot(j));
        sim_pos(:, j) = x_plot(1:2);
    end
    figure(paths);
    hold on;
    if i ~= num_sims + 1
        plot(sim_pos(1, :), sim_pos(2, :), "LineWidth", 2, "Color", [0 0.4470 0.7410 0.2]);
    else
        plot(sim_pos(1, :), sim_pos(2, :), "LineWidth", 2, "Color", [0.8500 0.3250 0.0980]);
    end
    scatter(x0_og(1), x0_og(2), "magenta", "filled");
    scatter(sim_pos(1, end), sim_pos(2, end), "magenta", "filled");
    if norm(sim_pos(1:2, end) - GuardParams.G{3}(1:2)) <= GuardParams.G{3}(3)
        sucesses = sucesses + 1;
    end
    hold off;
end
toc
disp(['Success: ', num2str(sucesses), ' simulations: ', num2str(num_sims + 1)]);
w_mag

%% Plot
% CEM
%AnimateBall(cem_sol, BallParams, GuardParams);

% MPC
%AnimateBall(mpc_sol, BallParams, GuardParams);

%PlotMPCSolutionAndSim(tmpc, xmpc, umpc, mpc_sol);
%domains
%AnimateMPC(tmpc, xmpc, BallParams, GuardParams);

% Both
%AnimateTwoBalls(mpc_sol, cem_sol, BallParams, GuardParams);

%% Controller Functions
function u = ZeroController()
    u = zeros(2, 1);
end

function uout = MpcController(t, x, u, tu)
    uout = interp1(tu(1:end-1), u', t)';
end

%% Plotting
function PlotMPCSolutionAndSim(tmpc, xmpc, umpc, sim_sol)

    tsim = linspace(sim_sol(1).x(1), sim_sol(end).x(end), 200);
    xsim = zeros(4, length(tsim));

    for i = 1:length(tsim)
       
        xsim(:, i) = GetState(sim_sol, tsim(i));
    end

    figure;
    plot(xmpc(1,:), xmpc(2,:), "LineWidth", 2);
    hold on;
    plot(xsim(1, :), xsim(2, :), "LineWidth", 2);
    hold off;

    figure;
    tiledlayout(3, 2);
    nexttile;
    hold on;
    plot(tmpc, xmpc(1, :));
    plot(tsim, xsim(1, :));
    hold off;
    legend(["mpc", "sim"]);
    title("x position")
    nexttile;
    hold on;
    plot(tmpc, xmpc(3, :));
    plot(tsim, xsim(3, :));
    hold off;
    legend(["mpc", "sim"]);
    title("x velocity")
    nexttile;
    hold on;
    plot(tmpc, xmpc(2, :));
    plot(tsim, xsim(2, :));
    hold off;
    legend(["mpc", "sim"]);
    title("y position")
    nexttile;
    hold on;
    plot(tmpc, xmpc(4, :));
    plot(tsim, xsim(4, :));
    hold off;
    legend(["mpc", "sim"]);
    title("y velocity")
    nexttile;
    plot(tmpc(1:end-1), umpc(1,:));
    nexttile;
    plot(tmpc(1:end-1), umpc(2,:));
    

end


function x = GetState(sol, t)
j = 1;
while sol(j).x(end) < t
    j = j + 1;
end

x = deval(sol(j), t);

end

function warm_start = CreateWarmstart(cem_solution, cem_controller, domains)
warm_start.x = zeros(4, sum(domains.nodes));
warm_start.u = zeros(2, sum(domains.nodes));    

t = linspace(0, sum(domains.T), length(warm_start.x));
for i = 1:length(warm_start.x)
    warm_start.x(:, i) = GetState(cem_solution, t(i));
    warm_start.u(:, i) = cem_controller.Compute(t(i), warm_start.x(:, i));
end

end

%% Plotting
function PlotSimAndTraj(sim_sol, tmpc, xmpc, umpc)
    for i = 1:2
        nexttile(3*i - 2);
        hold on;
        plot(sim_sol.x, sim_sol.y(i, :), "LineWidth", 2, "Color", [0 0.4470 0.7410]);
        plot(tmpc + sim_sol.x(1), xmpc(i, :), "LineStyle", "--", "LineWidth", 2, "Color", [0.8500 0.3250 0.0980]);
        scatter(sim_sol.x(1), sim_sol.y(i, 1), "magenta", "filled");
        hold off;
   
        nexttile(3*i - 1);
        hold on;
        plot(sim_sol.x, sim_sol.y(i + 2, :), "LineWidth", 2, "Color", [0 0.4470 0.7410]);
        plot(tmpc + sim_sol.x(1), xmpc(i + 2, :), "LineStyle", "--", "LineWidth", 2, "Color", [0.8500 0.3250 0.0980]);
        scatter(sim_sol.x(1), sim_sol.y(i + 2, 1), "magenta", "filled");
        hold off;

        nexttile(3*i);
        hold on;
        plot(tmpc(1:end-1) + sim_sol.x(1), umpc(i, :), "LineStyle", "--", "LineWidth", 2, "Color", [0.8500 0.3250 0.0980]);
        scatter(tmpc(1) + sim_sol.x(1), umpc(i, 1), "magenta", "filled");
        hold off;
    end
end


