function J = BodyJacobianFD(robot, q, joint, r)
pos_nom = ForwardKinematics(robot, q, zeros(robot.nv, 1), joint, r);

J = zeros(2, robot.nv);

FD_DELTA = 1e-8;

for i = 1:length(q)
    q(i) = q(i) + FD_DELTA;
    pos_pert = ForwardKinematics(robot, q, zeros(robot.nv, 1), joint, r);
    q(i) = q(i) - FD_DELTA;
    
    J(:, i) = (pos_pert - pos_nom)/FD_DELTA;
end

end

