function controller = CrossEntropyRobot(robot, costfcn, q0, v0, pos0, tf, cem_settings)
%CROSSENTROPY Uses CEM to design a controller/trajectory
%   
tic;

times = 0:cem_settings.dt:tf;
tau_nom = zeros(length(times), robot.nj_act);
trajectories.costs = 1e10*ones(cem_settings.samples, 1);
trajectories.torques = zeros(length(times), robot.nj_act, cem_settings.samples);
trajectories.elite_idx = zeros(cem_settings.num_elite, 1);
variance = cem_settings.var*ones(length(times), robot.nj_act);
min_variance = cem_settings.min_var*ones(length(times), robot.nj_act);
n = 0;
while n < cem_settings.max_iters && mean(mean(variance)) > cem_settings.terminal_eps
    n
    costs = trajectories.costs;
    torques_traj = trajectories.torques;
    parfor i = 1:cem_settings.samples
        %rng(i)

        % Generate random torques centered on a nominal trajectory
        torques = GenerateTorques(variance, tau_nom);
        controller_int = struct();
        controller_int.Compute = @(t,q, v, controller) interp1(times, torques, t);
        
        % Forward simulate the trajectories
        [t, q, qd, pos, te, qe, qde] = RobotSim(robot, q0, v0, pos0, 0, tf, 50, controller_int);
        %t = linspace(0, tf, 100);
        %q = randn(1)*ones(robot.nq, length(times));
        %qd = q;
        %pos = zeros(2, length(times));
        % Compute cost
        cost = costfcn.ComputeCost(t, q, qd, pos);
        % Insert trajectory
        costs(i) = cost;
        torques_traj(:, :, i) = torques;
        %trajectories = InsertTraj(trajectories, cost, torques);
    end
    trajectories.costs = costs;
    trajectories.torques = torques_traj;

    % TODO: remove and use a sort in the same order
    trajectories.elite_idx = UpdateEliteIdx(trajectories, cem_settings.num_elite);
    trajectories.elite_idx
    lowest_cost = trajectories.costs(trajectories.elite_idx(1))
    %trajectories.costs
    
    % Update variance and nominal traj
    [variance, tau_nom] = UpdateVarAndMean(cem_settings.num_elite, trajectories);

    cost_mean = GetEliteCostMean(cem_settings.num_elite, trajectories);
    cost_mean
    mean(mean(variance))
    variance = max(variance, min_variance);
    
    % TODO: Clear trajectories struct
    trajectories.costs = 1e10*ones(cem_settings.samples, 1);
    trajectories.torques = zeros(length(times), robot.nj_act, cem_settings.samples);

    % Repeat until some threshold has been achieved or a max iter count is
    % reached
    n = n + 1;
end

controller.Compute = @(t,q, v, controller) interp1(times, tau_nom, t);
toc

end

function torques = GenerateTorques(variance, tau_nom)
    torques = tau_nom;
    torques = torques + randn(size(tau_nom)).*sqrt(variance);
    for i = 1:size(tau_nom, 1)
        torques(i, :) = torques(i, :) + randn(1, size(tau_nom, 2)).*(variance(i, :).^(0.5));
    end
end

function trajectories = InsertTraj(trajectories, cost, torques)
    for i = 1:length(trajectories.costs)
        if cost < trajectories.costs(i)
            trajectories.costs(i) = cost;
            trajectories.torques(:, :, i) = torques;
            break;
        end
    end
    if i == length(trajectories.costs)
        warning("Provided traj did not have a lower cost compared to any in the struct.");
    end
end

function [var, mean] = UpdateVarAndMean(num_elite, trajectories)
    mean = zeros(size(trajectories.torques, 1), size(trajectories.torques, 2));
    var = zeros(size(trajectories.torques, 1), size(trajectories.torques, 2));

    % Mean
    for i = 1:num_elite
        mean = mean + trajectories.torques(:, :, trajectories.elite_idx(i));
    end
    mean = mean/num_elite;

    % Variance
    for i = 1:num_elite
        var = var + (trajectories.torques(:, :, trajectories.elite_idx(i)) - mean).^2;
    end
    var = var/num_elite;
end

function mean = GetEliteCostMean(num_elite, trajectories)
    mean = 0;
    for i = 1:num_elite
        mean = mean + trajectories.costs(trajectories.elite_idx(i));
    end

    mean = mean/num_elite;
end

function elite_idx = UpdateEliteIdx(trajectories, num_elite)
    elite_idx = zeros(num_elite, 1);
    num_less_than = zeros(length(trajectories.costs), 1);
    for i = 1:length(trajectories.costs)
        cost = trajectories.costs(i);
        for j = 1:length(trajectories.costs)
            if (cost > trajectories.costs(j))
                num_less_than(i) = num_less_than(i) + 1;
            end
        end
    end

    for i = 1:num_elite
        for j = 1:length(trajectories.costs)
            if num_less_than(j) == i-1
                elite_idx(i) = j;
            end
        end
    end
end

