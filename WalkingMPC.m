function [t, q, v, pos, torques, qe, ve, pos_e, cost] = WalkingMPC(robot, q0, v0, pos0, swing_params, costfcn)
%WALKINGMPC Summary of this function goes here
%   
import casadi.*

t = 0;
q = 0;
v = 0;
torques = 0;

% TODO: Fix
% uguess = zeros(numinputs, 1);
% xguess = zeros(numstates, 1);

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

% Create direct multiple shooting NLP
count = 1;
for swing = 1:swing_params.num_swings
    swing_times = linspace(0, swing_params.tf, swing_params.nodes);
    dt = swing_params.tf/swing_params.nodes;
    for k = 1:swing_params.nodes
        t_swing = (k-1)*dt;
        % New NLP variable for the control
        Uk = MX.sym(['U_' num2str((swing-1)*k)], 4);
        w = {w{:}, Uk};
        lbw = [lbw; -robot.torque_lims];
        ubw = [ubw; robot.torque_lims];
        w0 = [w0;  ones(robot.nj_act, 1)];
    
        % Integrate till the end of the interval
        Xk_end = DiscreteDynamics(Xk, Uk, dt, robot);
        J = J + costfcn(Xk_end, Uk);

        % New NLP variable for state at end of interval
        Xk = MX.sym(['X_' num2str((swing-1)*(k+1))], node_qvp);
        w = [w, {Xk}];
        lbw = [lbw; -[robot.joint_bounds; robot.joint_vel_bounds; inf; inf]];
        ubw = [ubw; [robot.joint_bounds; robot.joint_vel_bounds; inf; inf]];
        w0 = [w0; ones(node_qvp, 1)];
    
        % Add equality constraint for the shooting nodes
        g = [g, {Xk_end-Xk}];
        lbg = [lbg; zeros(node_qvp,1)];
        ubg = [ubg; zeros(node_qvp,1)];
        
        if (k > 1)
            % Add equality constraint for the swing foot
            fk_pos = ForwardKinematicsCasadi(robot, Xk, robot.swing, robot.foot_r);
            des_swing_pos = SwingTrajectory(t_swing, 0, swing_params.tf, 0, swing_params.length, swing_params.apex, 0, 0)
            g = [g, {fk_pos - des_swing_pos}];
            lbg = [lbg; zeros(2,1)];
            ubg = [ubg; zeros(2,1)];
            %g = [g, {fk_pos(2) - des_swing_pos(2)}];
            %lbg = [lbg; zeros(1,1)];
            %ubg = [ubg; zeros(1,1)];
        end
        
        if k == swing_params.nodes
            % New variable for the result of the impact map
            reset_result = ImpactMap(Xk, robot);
            Xk2 = MX.sym(['X_impact_' num2str(swing)], node_qvp);
            w = {w{:}, Xk2};
            lbw = [lbw; -inf*ones(node_qvp,1)];
            ubw = [ubw; inf*ones(node_qvp,1)];
            w0 = [w0; ones(node_qvp, 1)];

            g = [g, {Xk2(1:node_qvp-2) - reset_result}];
            %g = [g, {Xk2(1:node_qvp-2) - Xk(1:node_qvp-2)}];
            g = [g, {Xk2(robot.nq + robot.nv + 1:end) - Xk(robot.nq + robot.nv + 1:end)}];
            lbg = [lbg; zeros(node_qvp,1)];
            ubg = [ubg; zeros(node_qvp,1)];
        else 
            t = [t; t(end) + dt];
        end

        count = count + 1;
    end 
end
count

% Create an NLP solver
prob = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}));
solver = nlpsol('solver', 'ipopt', prob);

% Solve the NLP
sol = solver('x0', w0, 'lbx', lbw, 'ubx', ubw,...
            'lbg', lbg, 'ubg', ubg);
w_opt = full(sol.x);
cost = full(sol.f);

[q, v, pos, torques, qe, ve, pos_e] = ExtractValues(w_opt, robot, swing_params);

end

% TODO: Check this function
function [q, v, pos, torques, qe, ve, pos_e] = ExtractValues(w_opt, robot, swing_params)
    q = [];
    v = [];
    pos = [];
    torques = [];
    qe = [];
    ve = [];
    pos_e = [];

    idx = 1;
    for swing = 1:swing_params.num_swings
        for i = 1:swing_params.nodes
            q = [q, w_opt(idx:(idx-1) + robot.nq)];
            idx = idx + robot.nq;
            v = [v, w_opt(idx:(idx-1) + robot.nv)];
            idx = idx + robot.nv;
            pos = [pos, w_opt(idx:(idx-1) + 2)];
            idx = idx + 2;
            if idx < length(w_opt) - 4
                torques = [torques, w_opt(idx:idx + robot.nj_act)];
                idx = idx + robot.nj_act;
            end
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
    vplus = (eye(robot.nq) - Minv*Jeval'*inv(Jeval*Minv*Jeval')*Jeval)*v;

    % Now switch the joints
    % Calculate angle to the ground
    pos_joint = ForwardKinematicsCasadi(robot, x, robot.swing, [0;0]);
    pos_foot = ForwardKinematicsCasadi(robot, x, robot.swing, robot.foot_r);
    pos = pos_joint - pos_foot;
    q(1) = -atan2(pos(1), pos(2)); %-atan(pos(1)/pos(2));
    q(2) = -q(5);
    q(3) = -q(4);

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

    %qc = MX.sym('qc', robot.nq);
    %vc = MX.sym('qc', robot.nv);
    a = FDabCasadi(robot, q, v, [0; u]);
    
    xkp1 = x + dt*[v; a; fkjac(x)*x];
end

function [pos, vel] = ForwardKinematicsCasadi(robot, x, joint, r)
%FORWARDKINEMATICS Computes the forward kinematics for a given joint, and
% optionaly, for a frame at an fixed offset from the joint (given by theta
% and r)
%   Returns the location in the global frame and the velocity in the local
%   frame
import casadi.*

pos = MX.sym('pos', 2);
vel = MX.sym('vel', 2);

pos = r;

% Traverse the kinematic tree
% For now do it naively and compute the transform for every point

% Start with a position in the desired joint's frame
% Then use the transforms to back it out to the global frame

%r = [1; r(1); r(2)]; %[r; 0];
while (joint ~= 0)  % Go through all rotational joints
    [Xj, S] = jcalc(robot.jtype{joint}, x(joint));
    % joint
    % Xj
    % robot.Xtree{joint}
    Xup = Xj*robot.Xtree{joint};
    [theta, d] = plnr(Xup);

    temp = rz(-theta)*[pos; 0];
    pos = temp(1:2);
    pos = pos + d;

    joint = robot.parent(joint);
end
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
qdd = qd;
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
C = qd;
H = qd*qd';

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

