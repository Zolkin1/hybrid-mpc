function [pos, vel] = ForwardKinematicsCasadi(robot, x, joint, r)
%FORWARDKINEMATICS Computes the forward kinematics for a given joint, and
% optionaly, for a frame at an fixed offset from the joint (given by theta
% and r)
%   Returns the location in the global frame and the velocity in the local
%   frame
import casadi.*

%pos = MX.sym('pos', 2);
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
