function [q, cost] = InverseKinematicsRobot(robot, joint, r, q0, ee_pos)
%INVERSEKINEMATICS Computes joint angles given an end effector location
%   

q = q0;
cost = 10;

while cost > 1e-2
    cost = IKCost(robot, joint, r, q, ee_pos);
    
    % Compute the Jacobian at the current estimate
    J = BodyJacobianFD(robot, q, joint, r);
    
    % Compute inverse of Jacobian
    pos = ForwardKinematics(robot, q, zeros(robot.nv,1), joint, r);
    dqkp1 = pinv(J)*(ee_pos - pos);
    
    % Line search
    cost_new = IKCost(robot, joint, r, q + dqkp1, ee_pos);
    % TODO: Choose better termination conditions
    alpha = 0.5;
    beta = 0.9;
    grad = CostGrad(robot, joint, r, q, ee_pos);
    while alpha > 1e-3 && cost + beta*alpha*grad'*dqkp1 < cost_new
        dqkp1 = alpha * dqkp1;
        cost_new = IKCost(robot, joint, r, q + dqkp1, ee_pos);
        alpha = alpha*0.5;
    end

    if alpha <= 1e-3
        warning("Small alpha");
        q = q0;
        cost = inf;
        break;
    end
    
    q = q + dqkp1;
end

end

function cost = IKCost(robot, joint, r, q, ee_pos)
    fk_pos = ForwardKinematics(robot, q, zeros(robot.nv,1), joint, r);
    cost = norm(fk_pos - ee_pos);
end

function grad = CostGrad(robot, joint, r, q, ee_pos)
    grad = zeros(robot.nq, 1);
    FD_DELTA = 1e-8;

    for i = 1:robot.nq
        cost1 = IKCost(robot, joint, r, q, ee_pos);
        qpert = q;
        qpert(i) = qpert(i) + FD_DELTA;
        cost2 = IKCost(robot, joint, r, qpert, ee_pos);
        grad(i) = (cost2 - cost1)/FD_DELTA;
    end
end