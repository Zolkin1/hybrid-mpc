function [t, qout, qdout, pos, te, qe, qde, pos_e] = RobotSim(robot, q, qd, pos0, t0, tf, tdensity, controller)
%SIMULATE Simulates a robot
%   

% Forward integrate with ODE 45

y0 = [q; qd; pos0];

% E = odeEvent(EventFcn=@(t,y)GroundContact(t,y,robot),...
%         Direction="descending", ...
%         Response="callback", ...
%         CallbackFcn=@(t,y)ContactResponse(t,y,robot));
% F = ode(ODEFcn=@(t, y) odefun(t, y, robot, controller), InitialValue=y0, EventDefinition=E);
% S = solve(F, t0, tf, Refine=1);

t = linspace(t0, tf, floor((tf-t0)*tdensity)-1);
qout = zeros(robot.nq, length(t));
qdout = zeros(robot.nv, length(t));
pos = zeros(2, length(t));
t_idx = 1;

te = [];
qe = [];
qde = [];
pos_e = [];
robot.last_contact = -1;
options = odeset('Events', @(t,y)GroundContact(t,y,robot),'Refine',8);
global last_contact_time;

while t0 < tf
    sol = ode45(@(t, y) odefun(t, y, robot, controller), [t0, tf], y0, options);
    if (sol.x(length(sol.x)) < tf) 
        t0 = sol.xe(1);
        te = [te; t0];
        if abs(t0 - robot.last_contact) < 1e-2
            warning("Robot contacts too close!");
        end
        
        yf = deval(sol, sol.xe(1));
        %GroundContact(t0,yf,robot)

        %qf = yf(1:robot.nq);
        %vf = yf(robot.nq + 1: robot.nq + robot.nv);
        %[postemp, vel] = ForwardKinematics(robot, qf, vf, robot.swing, [0;-robot.calf_length]);
        %postemp(2)

        [stop, y0, robot, controller] = ContactResponse(sol.xe(1), yf, robot, controller);

        %GroundContact(t0,y0,robot)

        q = y0(1:robot.nq);
        v = y0(robot.nq+1:robot.nq+robot.nv);
        qe = [qe, q];
        qde = [qde, v];

        pos_e = [pos_e, y0(robot.nq+robot.nv+1:end)];
        last_contact_time = t0;
    else
        t0 = sol.x(length(sol.x));
    end
    
    while (t(t_idx) < sol.x(length(sol.x)))
        y_t = deval(sol, t(t_idx));
        qout(:, t_idx) = y_t(1:robot.nq);
        qdout(:, t_idx) = y_t(robot.nq + 1:robot.nq + robot.nv);
        pos(:, t_idx) = y_t(robot.nq + robot.nv + 1:end);
        t_idx = t_idx + 1;
    end
end

t = t(1:end-1);
qout = qout(:, 1:end-1);
qdout = qdout(:, 1:end-1);
pos = pos(:, 1:end-1);

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

function [stop, y, robot, controller] = ContactResponse(t, y, robot, controller)
    % Two options:
    % (a) Change which leg is swing vs stance by updating the kinematic
    % constraint
    % (b) Switch the joint angles and update the global positions. Keep
    % the same stance and swing feet

    %disp("Contact");
    %t

    q = y(1:robot.nq);
    v = y(robot.nq+1:robot.nq+robot.nv);

    % Reset map is given by
    % q+ = (I - M^-1*J^T*(J*M^-1*J^T)^-1*J)q-
    J = BodyJacobianFD(robot, q, robot.swing, robot.foot_r);
    [M, C] = HandC(robot, q, v, {});
    Minv = inv(M);
    vplus = (eye(robot.nq) - Minv*J'*inv(J*Minv*J')*J)*v;

    %J*v
    %J*vplus

    % Now switch the joints
    % Calculate angle to the ground
    pos_joint = ForwardKinematics(robot, q, v, robot.swing, [0;0]);
    pos_foot = ForwardKinematics(robot, q, v, robot.swing, robot.foot_r);
    pos = pos_joint - pos_foot;
    q(1) = -atan2(pos(1), pos(2)); %-atan(pos(1)/pos(2));
    q(2) = -q(5);
    q(3) = -q(4);

    q(4) = -y(3);
    q(5) = -y(2);

    % Remap velocities
    vplus_temp = vplus;
    % TODO: What is the rotational velocity of the new ground contact?
    % TODO: Remove
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

    % Also, apply the reset map
    % TODO: Add in the real reset map, for now set all velocities to 0
    % y(robot.nq + 1: robot.nq + robot.nv) = zeros(robot.nv, 1);
    % y = zeros(length(y), 1);
    % y(1) = 0.0;
    % y(2) = 0.8;
    % y(4) = 0.6;
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
        FDab(robot, q, v, [0, tau_act(1), tau_act(2), tau_act(3), tau_act(4)]);

    J = BodyJacobianFD(robot, q, 3, [0;0]);
    dydt(2*robot.nv+1:end) = J*v;
    % TODO: Do torso acc via casadi
end

