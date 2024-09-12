function J = BodyJacobian(robot, q, joint)
%BODYJACOBIAN Computes the Jacobian
%   Detailed explanation goes here

J = zeros(3, robot.nv);

X = eye(3);

while (joint ~= 0)
    joint
    [Xj, S] = jcalc(robot.jtype{joint}, q(joint));
    %robot.Xtree{robot.parent(joint)}
    Xj
    S
    J(:, joint) = X * S;
    X = X * robot.Xtree{joint} * Xj;
    %J(:,joint) = Xj*S;
    joint = robot.parent(joint);
end

end

