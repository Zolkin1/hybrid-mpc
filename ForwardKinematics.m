function [pos, vel] = ForwardKinematics(robot, q, qd, joint, r)
%FORWARDKINEMATICS Computes the forward kinematics for a given joint, and
% optionaly, for a frame at an fixed offset from the joint (given by theta
% and r)
%   Returns the location in the global frame and the velocity in the local
%   frame

pos = zeros(2, 1); %robot.nq);
vel = zeros(2, 1); %robot.nv);

% Traverse the kinematic tree
% For now do it naively and compute the transform for every point

% Start with a position in the desired joint's frame
% Then use the transforms to back it out to the global frame

%r = [1; r(1); r(2)]; %[r; 0];
while (joint ~= 0)  % Go through all rotational joints
    [Xj, S] = jcalc(robot.jtype{joint}, q(joint));
    % joint
    % Xj
    % robot.Xtree{joint}
    Xup = Xj*robot.Xtree{joint};
    [theta, d] = plnr(Xup);

    temp = rz(-theta)*[r; 0];
    r = temp(1:2);
    r = r + d;

    joint = robot.parent(joint);
end
pos = r;
end

