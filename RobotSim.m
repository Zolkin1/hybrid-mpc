function [t, qout, qdout, te, qe, qde] = RobotSim(robot, q, qd, t0, tf, tdensity, controller)
%SIMULATE Simulates a robot
%   

% Forward integrate with ODE 45

y0 = [q; qd];

% E = odeEvent(EventFcn=@(t,y)GroundContact(t,y,robot),...
%         Direction="descending", ...
%         Response="callback", ...
%         CallbackFcn=@(t,y)ContactResponse(t,y,robot));
% F = ode(ODEFcn=@(t, y) odefun(t, y, robot, controller), InitialValue=y0, EventDefinition=E);
% S = solve(F, t0, tf, Refine=1);

t = linspace(t0, tf, floor((tf-t0)*tdensity));
qout = zeros(robot.nq, length(t));
qdout = zeros(robot.nv, length(t));
t_idx = 1;

te = [];
qe = zeros(robot.nq, 1);
qde = zeros(robot.nq, 1);

options = odeset('Events', @(t,y)GroundContact(t,y,robot),'Refine',4);
while t0 < tf
    sol = ode45(@(t, y) odefun(t, y, robot, controller), [t0, tf], y0, options);
    if (sol.x(length(sol.x)) < tf) 
        t0 = sol.xe;
        te = [te; t0];
        
        yf = deval(sol, sol.xe);
        [stop, y0] = ContactResponse(sol.xe, yf, robot);
    else
        t0 = sol.x(length(sol.x));
    end
    
    while (t(t_idx) < sol.x(length(sol.x)))
        y_t = deval(sol, t(t_idx));
        qout(:, t_idx) = y_t(1:robot.nq);
        qdout(:, t_idx) = y_t(robot.nq + 1:robot.nq + robot.nv);
        t_idx = t_idx + 1;
    end
end

end

function [height, isterminal, direction] = GroundContact(t, y, robot)
    % Return distance of foot from ground
    q = y(1:robot.nq);
    v = y(robot.nq + 1: robot.nq + robot.nv);
    swing_idx = robot.leg_1;
     if (robot.stance == robot.leg_1)
        swing_idx = robot.leg_2;        
    end
    [pos, vel] = ForwardKinematics(robot, q, v, swing_idx, [0;-robot.calf_length]);

    height = pos(2);
    isterminal = 1;
    direction = -1;
end

function [stop, y] = ContactResponse(t, y, robot)
    % Two options:
    % (a) Change which leg is swing vs stance by updating the kinematic
    % constraint
    % (b) Switch the joint angles and update the global positions. Keep
    % the same stance and swing feet

    if (robot.stance == robot.leg_1)
        robot.stance = robot.leg_2;
        % TODO: Remove these
        y(4) = 0.6;
        y(6) = 0;
    else
        robot.stance = robot.leg_1;
        y(4) = 0;
        y(6) = 0.6;
    end

    disp("Contact");

    % Also, apply the reset map
    % TODO: Add in the real reset map, for now set all velocities to 0
    y(robot.nq + 1: robot.nq + robot.nv) = zeros(robot.nv, 1);
    y(2) = 1;
    stop = false;
end

function dydt = odefun(t, y, robot, controller)
    dydt = zeros(2*robot.nv, 1);
    q = y(1:robot.nq);
    v = y(robot.nq + 1: robot.nq + robot.nv);
    dydt(1:robot.nv) = v;
    dydt(robot.nv + 1: 2*robot.nv) = ...
        FDab(robot, q, v, [0; 0; 0; ...
        controller.Compute(t, q(3:2+robot.nj_act), v(3:2+robot.nj_act), controller)]);
end