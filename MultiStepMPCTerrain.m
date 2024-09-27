function mpc_sol = MultiStepMPCTerrain(robot, q0, v0, pos0, swing_params, ground, costfcn, warm_start)
%MULTISTEPMPC
%   
import casadi.*

t = 0; %swing_params.time_into_swing;
q = 0;
v = 0;
torques = 0;
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

node_qvp = robot.nq + robot.nv + 2;

x0 = [q0; v0; pos0];

% IC Constraint
Xk = MX.sym('X0', node_qvp);
w = {w{:}, Xk};
lbw = [lbw; x0];
ubw = [ubw; x0];
w0 = [w0; x0];

current_foot_pos = ForwardKinematicsCasadi(robot, Xk, robot.swing, robot.foot_r);

% Create direct multiple shooting NLP
count = 1;
var_idx = 1;
ws_node_idx = 1;
for swing = 1:swing_params.num_swings
    % Swing parameter variables
    swing_tf = MX.sym(['SwingTf_' num2str(swing)], 1);
    w = {w{:}, swing_tf};
    lbw = [lbw; 0.1];
    ubw = [ubw; 0.5];
    w0 = [w0;  0.3];

    swing_xpos = MX.sym(['SwingLength_' num2str(swing)], 1);
    w = {w{:}, swing_xpos};
    lbw = [lbw; -0.5];
    ubw = [ubw; 0.5];
    w0 = [w0;  0.3];

    swing_times = linspace(0, swing_tf, swing_params.nodes(swing));
    dt = swing_tf/swing_params.nodes(swing);
    start_node = 1;
    for k = start_node:swing_params.nodes(swing)
        t_swing = (k)*dt + swing_params.time_into_swing;
        % New NLP variable for the control
        Uk = MX.sym(['U_' num2str(var_idx)], robot.nj_act);
        var_idx = var_idx + 1;
        w = {w{:}, Uk};
        lbw = [lbw; -robot.torque_lims];
        ubw = [ubw; robot.torque_lims];
        w0 = [w0;  warm_start.tau(:, ws_node_idx)];
    
        % ---------- Dynamics ---------- %
        Xk_end = DiscreteDynamics(Xk, Uk, dt, robot);

        % ---------- Cost Function ---------- %
        J = J + costfcn(Xk_end, Uk, t_swing, dt);

        % New NLP variable for state at end of interval
        Xk = MX.sym(['X_' num2str(var_idx)], node_qvp);
        var_idx = var_idx + 1;
        w = [w, {Xk}];
        lbw = [lbw; [robot.joint_bounds_lower; -robot.joint_vel_bounds; -inf; -inf]];
        ubw = [ubw; [robot.joint_bounds_upper; robot.joint_vel_bounds; inf; inf]];
        w0 = [w0; [warm_start.q(:, ws_node_idx);...
            warm_start.v(:, ws_node_idx); warm_start.pos(:, ws_node_idx)]];
    
        % Add equality constraint for the shooting nodes
        g = [g, {Xk_end-Xk}];
        lbg = [lbg; zeros(node_qvp,1)];
        ubg = [ubg; zeros(node_qvp,1)];
        
        % ---------- Swing Height ---------- %
        if (k > 2 && swing ~= swing_params.no_swing_constraint)
            start_height = ground.start_height(swing_params.ground_idx(swing));
            end_height = ground.end_height(swing_params.ground_idx(swing));

            apex = max(start_height, end_height) + 0.08;

            des_swing_pos = SwingTrajectoryCasadi(t_swing, k, swing_params.nodes(swing), 0, swing_tf, current_foot_pos(1), ...
                swing_xpos, apex, start_height, end_height);

            % Add equality constraint for the swing foot
            fk_pos = ForwardKinematicsCasadi(robot, Xk, robot.swing, robot.foot_r);
            g = [g, {fk_pos - des_swing_pos}];
            lbg = [lbg; zeros(2,1)];
            ubg = [ubg; zeros(2,1)];
        elseif k > 1
            fk_pos = ForwardKinematicsCasadi(robot, Xk, robot.swing, robot.foot_r);
            g = [g, {fk_pos(2)}];
            lbg = [lbg; 0.02*ones(1,1)];
            ubg = [ubg; inf*ones(1,1)];
        end

        % ---------- Joint Position Constraints ---------- %
        % Keep the knee some amount above the the ground
        swing_knee_pos = ForwardKinematicsCasadi(robot, Xk, 4, [0; 0]);
        g = [g, {swing_knee_pos(2)}];
        lbg = [lbg; robot.calf_length/2];
        ubg = [ubg; inf*ones(1,1)];

        stance_knee_pos = ForwardKinematicsCasadi(robot, Xk, 2, [0; 0]);
        g = [g, {stance_knee_pos(2)}];
        lbg = [lbg; robot.calf_length/2];
        ubg = [ubg; inf*ones(1,1)];

        % ---------- Time Update ---------- %
        if isempty(te)
            t = [t; k*dt];
        else
            t = [t; k*dt + te(end)];
        end

        if k == swing_params.nodes(swing)
            % ---------- Guard Constraint ---------- %
            g = [g, {Xk(robot.nq + 1:robot.nq + robot.nv)}];
            lbg = [lbg; zeros(robot.nv, 1)];
            ubg = [ubg; zeros(robot.nv, 1)];

            % ---------- Impact Map ---------- %
            % New variable for the result of the impact map
            reset_result = ImpactMap(Xk, robot);
            temp = Xk;
            Xk = MX.sym(['X_impact_' num2str(var_idx)], node_qvp);
            var_idx = var_idx + 1;
            w = {w{:}, Xk};
            lbw = [lbw; -inf*ones(node_qvp,1)];
            ubw = [ubw; inf*ones(node_qvp,1)];
            w0 = [w0; [warm_start.q(:, ws_node_idx);...
                warm_start.v(:, ws_node_idx); warm_start.pos(:, ws_node_idx)]];

            g = [g, {Xk(1:node_qvp-2) - reset_result}];
            g = [g, {Xk(robot.nq + robot.nv + 1:end) - temp(robot.nq + robot.nv + 1:end)}];
            lbg = [lbg; zeros(node_qvp,1)];
            ubg = [ubg; zeros(node_qvp,1)];

            current_foot_pos = ForwardKinematicsCasadi(robot, Xk, robot.swing, robot.foot_r);
            if isempty(te)
                te = [te; k*dt + dt];
            else
                te = [te; k*dt + te(end) + dt];
            end
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

[q, v, pos, u, qe, ve, pos_e, swing_tfs, swing_xpos] = ExtractValues(w_opt, robot, swing_params);

mpc_sol.q = q;
mpc_sol.v = v;
mpc_sol.pos = pos;
mpc_sol.u = u;
mpc_sol.qe = qe;
mpc_sol.ve = ve;
mpc_sol.pos_e = pos_e;
mpc_sol.cost = cost;
mpc_sol.swing_tf = swing_tfs;
mpc_sol.swing_xpos = swing_xpos;

t = 0;
te = [];
for swing = 1:swing_params.num_swings
    dt = mpc_sol.swing_tf(swing)/swing_params.nodes(swing);
    for k = 1:swing_params.nodes(swing)
        if isempty(te)
            t = [t; k*dt];
        else
            t = [t; k*dt + te(end)];
        end

        if k == swing_params.nodes(swing)
            if isempty(te)
                te = [te; k*dt + dt];
            else
                te = [te; k*dt + te(end) + dt];
            end
        end
    end
end

mpc_sol.t = t;
mpc_sol.te = te;

end

function [q, v, pos, torques, qe, ve, pos_e, swing_tfs, swing_xpos] =...
    ExtractValues(w_opt, robot, swing_params)
    q = [];
    v = [];
    pos = [];
    torques = [];
    qe = [];
    ve = [];
    pos_e = [];
    swing_tfs = zeros(swing_params.num_swings, 1);
    swing_xpos = zeros(swing_params.num_swings, 1);

    idx = 1;

    % Initial condition
    q = [q, w_opt(idx:(idx-1) + robot.nq)];
    idx = idx + robot.nq;
    v = [v, w_opt(idx:(idx-1) + robot.nv)];
    idx = idx + robot.nv;
    pos = [pos, w_opt(idx:(idx-1) + 2)];
    idx = idx + 2;

    for swing = 1:swing_params.num_swings
        swing_tfs(swing) = w_opt(idx);
        idx = idx + 1;

        swing_xpos(swing) = w_opt(idx);
        idx = idx + 1;

        start_node = 1;
        for i = start_node:swing_params.nodes(swing)
            %if idx < length(w_opt) - 4
            torques = [torques, w_opt(idx:(idx-1) + robot.nj_act)];
            idx = idx + robot.nj_act;
            %end
            q = [q, w_opt(idx:(idx-1) + robot.nq)];
            %disp(i)
            %q(:, end)
            idx = idx + robot.nq;
            v = [v, w_opt(idx:(idx-1) + robot.nv)];
            idx = idx + robot.nv;
            pos = [pos, w_opt(idx:(idx-1) + 2)];
            idx = idx + 2;
            %idx
        end
        qe = [qe, w_opt(idx:(idx-1) + robot.nq)];
        idx = idx + robot.nq;
        ve = [ve, w_opt(idx:(idx-1) + robot.nv)];
        idx = idx + robot.nv;
        pos_e = [pos_e, w_opt(idx:(idx-1) + 2)];
        idx = idx + 2;
    end

    if idx ~= length(w_opt) + 1
        length(w_opt)
        idx
        warning("idx is not at the correct value!");
    end
end

function xkp1 = ImpactMap(x, robot)
    import casadi.*
    q = x(1:robot.nq);
    v = x(robot.nq+1:robot.nq+robot.nv);

    % Reset map is given by
    % q+ = (I - M^-1*J^T*(J*M^-1*J^T)^-1*J)q-
    pos = ForwardKinematicsCasadi(robot, x, robot.swing, robot.foot_r);
    J = jacobian(pos, x);
    fkjac = Function('Jacobian', {x}, {J});
    Jeval = fkjac(x);
    Jeval = Jeval(:,1:robot.nq);

    [M, C] = HandCCasadi(robot, q, v, {});
    Minv = inv(M);
    vplus = (eye(robot.nq) - Minv*Jeval'*inv(Jeval*Minv*Jeval')*Jeval)*x(robot.nq+1:robot.nq+robot.nv);
    %vplus = v;

    % Now switch the joints
    % Calculate angle to the ground
    pos_joint = ForwardKinematicsCasadi(robot, x, robot.swing, [0;0]);
    pos_foot = ForwardKinematicsCasadi(robot, x, robot.swing, robot.foot_r);
    pos = pos_joint - pos_foot;
    q(1) = -atan2(pos(1), pos(2)); %-atan(pos(1)/pos(2));
    q(2) = -x(5);
    q(3) = -x(4);
     
    q(4) = -x(3);
    q(5) = -x(2);

    % Remap velocities
    vplus_temp = vplus;
    % TODO: What is the rotational velocity of the new ground contact?
    % TODO: Remove
    %vplus(1) = -vplus(1); %0.5;
    vplus(2) = -vplus(5);
    vplus(3) = -vplus(4);
    vplus(4) = -vplus_temp(3);
    vplus(5) = -vplus_temp(2);

    % Assign to y
    xkp1 = [q; vplus];
end

function xkp1 = DiscreteDynamics(x, u, dt, robot)
    import casadi.*

    q = x(1:robot.nq);
    v = x(robot.nq + 1: robot.nq + robot.nv);
    
    %r = MX.sym('foot_r', 2);
    %pos = TestFcn(x);
    pos = ForwardKinematicsCasadi(robot, x, 3, [0;0]);
    J = jacobian(pos, x);
    fkjac = Function('Jacobian', {x}, {J});
    Jvel = fkjac(x);
    Jvel = Jvel(:, 1:robot.nq);

    %qc = MX.sym('qc', robot.nq);
    %vc = MX.sym('qc', robot.nv);
    a = FDabCasadi(robot, q, v, u); % [0; u]
    
    xkp1 = x + dt*[x(robot.nq + 1: robot.nq + robot.nv); a; Jvel*x(robot.nq + 1: robot.nq + robot.nv)];
end

function  qdd = FDabCasadi( model, q, qd, tau, f_ext )

% FDab  Forward Dynamics via Articulated-Body Algorithm
% FDab(model,q,qd,tau,f_ext,grav_accn)  calculates the forward dynamics of
% a kinematic tree via the articulated-body algorithm.  q, qd and tau are
% vectors of joint position, velocity and force variables; and the return
% value is a vector of joint acceleration variables.  f_ext is an optional
% argument specifying the external forces acting on the bodies.  It can be
% omitted if there are no external forces.  The format of f_ext is
% explained in the source code of apply_external_forces.
import casadi.*

a_grav = get_gravity(model);

% Just to initialize
qdd = 0*qd;
%qdd = MX.sym('qdd', length(qd));

for i = 1:model.NB
  [ XJ, S{i} ] = jcalc( model.jtype{i}, q(i) );
  vJ = S{i}*qd(i);
  Xup{i} = XJ * model.Xtree{i};
  if model.parent(i) == 0
    v{i} = vJ;
    c{i} = zeros(size(a_grav));		% spatial or planar zero vector
  else
    v{i} = Xup{i}*v{model.parent(i)} + vJ;
    c{i} = crm(v{i}) * vJ;
  end
  IA{i} = model.I{i};
  pA{i} = crf(v{i}) * model.I{i} * v{i};
end

if nargin == 5
  pA = apply_external_forces( model.parent, Xup, pA, f_ext );
end

for i = model.NB:-1:1
  U{i} = IA{i} * S{i};
  d{i} = S{i}' * U{i};
  u{i} = tau(i) - S{i}'*pA{i};
  if model.parent(i) ~= 0
    Ia = IA{i} - U{i}/d{i}*U{i}';
    pa = pA{i} + Ia*c{i} + U{i} * u{i}/d{i};
    IA{model.parent(i)} = IA{model.parent(i)} + Xup{i}' * Ia * Xup{i};
    pA{model.parent(i)} = pA{model.parent(i)} + Xup{i}' * pa;
  end
end

for i = 1:model.NB
  if model.parent(i) == 0
    a{i} = Xup{i} * -a_grav + c{i};
  else
    a{i} = Xup{i} * a{model.parent(i)} + c{i};
  end
  qdd(i) = (u{i} - U{i}'*a{i})/d{i};
  a{i} = a{i} + S{i}*qdd(i);
end
end

function  [H,C] = HandCCasadi( model, q, qd, f_ext )

% HandC  Calculate coefficients of equation of motion
% [H,C]=HandC(model,q,qd,f_ext)  calculates the coefficients of the
% joint-space equation of motion, tau=H(q)qdd+C(d,qd,f_ext), where q, qd
% and qdd are the joint position, velocity and acceleration vectors, H is
% the joint-space inertia matrix, C is the vector of gravity,
% external-force and velocity-product terms, and tau is the joint force
% vector.  Algorithm: recursive Newton-Euler for C, and
% Composite-Rigid-Body for H.  f_ext is an optional argument specifying the
% external forces acting on the bodies.  It can be omitted if there are no
% external forces.  The format of f_ext is explained in the source code of
% apply_external_forces.

import casadi.*
% Temporary for casadi to work well
C = 0*qd;
H = 0*qd*qd';

a_grav = get_gravity(model);

for i = 1:model.NB
  [ XJ, S{i} ] = jcalc( model.jtype{i}, q(i) );
  vJ = S{i}*qd(i);
  Xup{i} = XJ * model.Xtree{i};
  if model.parent(i) == 0
    v{i} = vJ;
    avp{i} = Xup{i} * -a_grav;
  else
    v{i} = Xup{i}*v{model.parent(i)} + vJ;
    avp{i} = Xup{i}*avp{model.parent(i)} + crm(v{i})*vJ;
  end
  fvp{i} = model.I{i}*avp{i} + crf(v{i})*model.I{i}*v{i};
end

if nargin == 4
  fvp = apply_external_forces( model.parent, Xup, fvp, f_ext );
end

for i = model.NB:-1:1
  C(i,1) = S{i}' * fvp{i};
  if model.parent(i) ~= 0
    fvp{model.parent(i)} = fvp{model.parent(i)} + Xup{i}'*fvp{i};
  end
end

IC = model.I;				% composite inertia calculation

for i = model.NB:-1:1
  if model.parent(i) ~= 0
    IC{model.parent(i)} = IC{model.parent(i)} + Xup{i}'*IC{i}*Xup{i};
  end
end

%H = zeros(model.NB);

for i = 1:model.NB
  fh = IC{i} * S{i};
  H(i,i) = S{i}' * fh;
  j = i;
  while model.parent(j) > 0
    fh = Xup{j}' * fh;
    j = model.parent(j);
    H(i,j) = S{j}' * fh;
    H(j,i) = H(i,j);
  end
end

end

