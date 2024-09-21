function [t, x, u, te, xe] = BouncingBallMPC(domains, costfcn, x0, BallParams, GuardParams)
%BOUNCINGBALLMPC 
%   

import casadi.*

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

if swing_params.num_swings ~= 1
    error("Number of swing must be 1 for the short horizon mpc!");
end

for dom = 1:domains.num
    for k = 1:domains.nodes(dom)
        % New NLP variable for the control
        Uk = MX.sym(['U_' num2str(var_idx)], robot.nj_act);
        var_idx = var_idx + 1;
        w = {w{:}, Uk};

        lbw = [lbw; -BallParams.ForceLim];
        if domains.type(dom) == 1
            ubw = [ubw; [0; 0]];
        else
            ubw = [ubw; BallParams.ForceLim];
        end
        w0 = [w0;  warm_start.tau(:, ws_node_idx)];
    
        % ---------- Dynamics ---------- %
        Xk_end = DiscreteDynamics(Xk, Uk, dt, domains.type(dom), BallParams);

        % ---------- Cost Function ---------- %
        J = J + costfcn(Xk_end, Uk, dt);

        % New NLP variable for state at end of interval
        Xk = MX.sym(['X_' num2str(var_idx)], node_qvp);
        var_idx = var_idx + 1;
        w = [w, {Xk}];
        lbw = [lbw; -inf*ones(node_states, 1)];
        ubw = [ubw; inf*ones(node_states, 1)];
        w0 = [w0; [warm_start.x(:, ws_node_idx)]];
    
        % Add equality constraint for the shooting nodes
        g = [g, {Xk_end-Xk}];
        lbg = [lbg; zeros(node_qvp,1)];
        ubg = [ubg; zeros(node_qvp,1)];
        
        if dom ~= domains.num && k == domains.nodes(dom)
            % ---------- Guard Constraints ---------- %
            g = [g, {Xk(domains.variable(dom)) - domains.guard_val(dom)}];
            lbg = [lbg; 0];
            ubg = [ubg; 0];

            if domains.variable(dom) == 1
                g = [g, {Xk(2)}];
            else
                g = [g, {Xk(1)}];
            end
            lbg = [lbg; min(domains.guard_constraint{dom})];
            ubg = [ubg; max(domains.guard_constraint{dom})];

            % ---------- Impact Map ---------- %
            if domains.guard_idx(dom) == 1 || domains.guard_idx(dom) == 2
                impact_result = WallReset(domains.guard_idx(dom), Xk, BallParams, GuardParams);
                Xk = MX.sym(['X_impact_' num2str(dom)], node_qvp);
                w = [w, {Xk}];
                lbw = [lbw; -inf*ones(node_states, 1)];
                ubw = [ubw; inf*ones(node_states, 1)];
                w0 = [w0; [warm_start.x(:, ws_node_idx)]];
    
                g = [g; {Xk - impact_result}];
                lbg = [lbg; zeros(node_states, 1)];
                ubg = [ubg; zeros(node_states, 1)];
            end
        end

        % ---------- Terminal Constraint ---------- %
        if dom == domains.num && k == domains.nodes(dom)
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

[x, xe] = ExtractValues(w_opt, robot, swing_params);

% TODO: Fill out return values

end

function xkp1 = DiscreteDynamics(x, u, dt, type, BallParams)
    xkp1 = zeros(length(x), 1);
    xkp1(1:2) = dt*x(3:4);
    if type == 1
        xkp1(3:4) = dt*BallParams.g + min(u, [0;0])/BallParams.M;
    else
        xkp1(3:4) = dt*u/BallParams.M;
    end
end

