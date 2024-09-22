function [t, x, u, te, xe] = BouncingBallMPC(domains, costfcn, x0, warm_start, BallParams, GuardParams)
%BOUNCINGBALLMPC 
%   

import casadi.*

t = 0;
te = [];

% Start with an empty NLP
w={};       % decision variables
w0 = [];    % initial guess
lbw = [];   % box constraints on the state and input (decision vars)
ubw = [];
J = 0;      % Cost function
g={};       % general constraints
lbg = [];   % lower and upper bound on the general constraint
ubg = [];

node_states = 4;
node_inputs = 2;

% warm_start.x = zeros(node_states, sum(domains.nodes));
% for i = 1:length(warm_start.x)
%     warm_start.x(:, i) = x0;
% end
% warm_start.u = zeros(node_inputs, sum(domains.nodes));

% IC Constraint
Xk = MX.sym('X0', node_states);
w = {w{:}, Xk};
lbw = [lbw; x0];
ubw = [ubw; x0];
w0 = [w0; x0];

% Create direct multiple shooting NLP
count = 1;
var_idx = 1;
ws_node_idx = 1;

for dom = 1:domains.num
    dt = domains.T(dom)/domains.nodes(dom);
    for k = 1:domains.nodes(dom)
        % New NLP variable for the control
        Uk = MX.sym(['U_' num2str(var_idx)], 2);
        var_idx = var_idx + 1;
        w = {w{:}, Uk};

        lbw = [lbw; -BallParams.ForceLim];
        if domains.type(dom) == 1
            ubw = [ubw; [0; 0]];
        else
            ubw = [ubw; BallParams.ForceLim];
        end
        w0 = [w0;  warm_start.u(:, ws_node_idx)];
    
        % ---------- Dynamics ---------- %
        Xk_end = DiscreteDynamics(Xk, Uk, dt, domains.type(dom), BallParams);

        % ---------- Cost Function ---------- %
        J = J + costfcn(Xk_end, Uk, dt);

        % New NLP variable for state at end of interval
        Xk = MX.sym(['X_' num2str(var_idx)], node_states);
        var_idx = var_idx + 1;
        w = [w, {Xk}];
        lbw = [lbw; -inf*ones(node_states, 1)];
        ubw = [ubw; inf*ones(node_states, 1)];
        w0 = [w0; [warm_start.x(:, ws_node_idx)]];
    
        % Add equality constraint for the shooting nodes
        g = [g, {Xk_end-Xk}];
        lbg = [lbg; zeros(node_states,1)];
        ubg = [ubg; zeros(node_states,1)];
        
        t = [t; t(end) + dt];

        % ---------- Guard Avoid Constraints ---------- %  
        % Avoid all the guards
        % if  k > 10 && ~(domains.guard_idx(dom) == 2) % k == domains.nodes(dom) && 
        %     g = [g, {Xk(1)}];
        %     lbg = [lbg; BallParams.r];
        %     ubg = [ubg; inf];
        % end
        % 
        % if  k > 10 && ~(domains.guard_idx(dom) == 1) % k == domains.nodes(dom) &&
        %     g = [g, {Xk(2)}];
        %     lbg = [lbg; BallParams.r];
        %     ubg = [ubg; inf];
        % end

        % if ~domains.guard_idx(dom) == 3
        %     g = [g, {norm(Xk(1:2) - GuardParams.G{3}(1:2)')}];
        %     lbg = [lbg; GuardParams.G{3}(3)];
        %     ubg = [ubg; inf];
        % end

        % if domains.type(dom) == 2
        %     g = [g, {dot(Xk(1:2) - GuardParams.G{3}(1:2)', Xk(1:2) - GuardParams.G{3}(1:2)')}];
        %     lbg = [lbg; -1];
        %     ubg = [ubg; GuardParams.G{3}(3)^2];
        % end

        if dom ~= domains.num && k == domains.nodes(dom)
            % g = [g, {[Xk(1); Xk(2)]}];
            % lbg = [lbg; [0; 0.1]];
            % ubg = [ubg; [10; 0.1]];
            % ---------- Guard Constraints ---------- %
            if domains.guard_idx(dom) == 1 || domains.guard_idx(dom) == 2
                g = [g, {Xk(domains.variable(dom)) - BallParams.r - domains.guard_val(dom)}];
                lbg = [lbg; 0];
                ubg = [ubg; 0];
    
                if domains.variable(dom) == 1
                    g = [g, {Xk(2)}];
                else
                    g = [g, {Xk(1)}];
                end
                lbg = [lbg; min(domains.guard_constraint{dom})];
                ubg = [ubg; max(domains.guard_constraint{dom})];
            else
                % g = [g, {norm(Xk(1:2) - GuardParams.G{3}(1:2)')}];
                % lbg = [lbg; GuardParams.G{3}(3)];
                % ubg = [ubg; GuardParams.G{3}(3)];
            end

            % ---------- Impact Map ---------- %
            if domains.guard_idx(dom) == 1 || domains.guard_idx(dom) == 2
                impact_result = WallReset(domains.guard_idx(dom), Xk, BallParams, GuardParams);
                Xk = MX.sym(['X_impact_' num2str(dom)], node_states);
                w = [w, {Xk}];
                lbw = [lbw; -inf*ones(node_states, 1)];
                ubw = [ubw; inf*ones(node_states, 1)];
                w0 = [w0; [warm_start.x(:, ws_node_idx)]];

                te = [te; domains.T(dom)];

                % TODO: Put back
                g = [g, {Xk - impact_result}];
                lbg = [lbg; zeros(node_states, 1)];
                ubg = [ubg; zeros(node_states, 1)];
            end
        end

        % ---------- Terminal Constraint ---------- %
        if dom == domains.num
        end

        ws_node_idx = ws_node_idx + 1;
        count = count + 1;
    end 
end

% Create an NLP solver
prob = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}));
solver = nlpsol('solver', 'ipopt', prob);

% Solve the NLP
sol = solver('x0', w0, 'lbx', lbw, 'ubx', ubw,...
            'lbg', lbg, 'ubg', ubg);
w_opt = full(sol.x);
cost = full(sol.f);

[x, u, xe] = ExtractValues(w_opt, domains);
end

%% Conversions
function [x, u, xe] = ExtractValues(w_opt, domains)

node_states = 4;
node_inputs = 2;

impact_idx = 1;
state_idx = 1;
input_idx = 1;
opt_idx = 1;

x = zeros(node_states, sum(domains.nodes));
xe = zeros(node_states, domains.num - 1);
u = zeros(node_inputs, sum(domains.nodes));

% Initial condition
x(:, state_idx) = w_opt(opt_idx:node_states);
state_idx = state_idx + 1;
opt_idx = opt_idx + node_states;

for dom = 1:domains.num
    for k = 1:domains.nodes(dom)
        u(:, input_idx) = w_opt(opt_idx : (opt_idx-1) + node_inputs);
        input_idx = input_idx + 1;
        opt_idx = opt_idx + node_inputs;

        x(:, state_idx) = w_opt(opt_idx : (opt_idx-1) + node_states);
        state_idx = state_idx + 1;
        opt_idx = opt_idx + node_states;
    end

    if dom ~= domains.num
        if domains.guard_idx(dom) == 1 || domains.guard_idx(dom) == 2 
            xe(:, impact_idx) = w_opt(opt_idx : (opt_idx-1) + node_states);
            impact_idx = impact_idx + 1;
            opt_idx = opt_idx + node_states;
        end
    end
end

if opt_idx <= length(w_opt)
    error("did not extract values properly")
end
end

%% Dynamics
function xkp1 = DiscreteDynamics(x, u, dt, type, BallParams)
    poskp1 = x(3:4);
    if type == 1
        velkp1 = BallParams.drag*(dot(x(3:4), x(3:4)))*x(3:4) + BallParams.g + u/BallParams.M;
    else
        velkp1 = BallParams.drag*(dot(x(3:4), x(3:4)))*x(3:4) + u/BallParams.M;
    end

    xkp1 = x + dt*[poskp1; velkp1];
end

