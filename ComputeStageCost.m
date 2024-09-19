function cost = ComputeStageCost(x, u, t, dt, robot, params)
    import casadi.*
    cost = PositionTracking(x(robot.nq + robot.nv + 1: end),...
        params.pos_target, params.pos_weight);

    cost = cost + PoseTracking(x(1:robot.nq), params.pose_target,...
       params.pose_weight);

    cost = cost + VelocityReg(x(robot.nq + 1:robot.nq + robot.nv),...
       params.vel_target, params.vel_weight);

    cost = cost + TorqueReg(u, params.u_target, params.u_weight);

    %cost = cost + FootTracking(x, t, curr_foot_pos, swing_num, params.swing_params, params.swing_weights, robot);

    cost = cost + HeadTracking(x, params.head_target, params.head_weight, robot);

    cost = cost*dt;
end

function cost = PositionTracking(pos, pos_target, pos_weight)
    cost = dot(pos_weight.*(pos - pos_target), pos_weight.*(pos - pos_target));
end

function cost = PoseTracking(q, q_target, q_weight)
    cost = dot(q_weight.*(q - q_target), q_weight.*(q - q_target));
end

function cost = VelocityReg(v, v_target, v_weight)
    cost = dot(v_weight.*(v - v_target), v_weight.*(v - v_target));
end

function cost = TorqueReg(u, u_target, u_weight)
    cost = dot(u_weight.*(u - u_target), u_weight.*(u - u_target));
end

function cost = FootTracking(x, t, curr_foot_pos, swing_num, params, weights, robot)
    fk_pos = ForwardKinematicsCasadi(robot, x, robot.swing, robot.foot_r);
    des_swing_pos = SwingTrajectory(t, 0, params.tf(swing_num), curr_foot_pos(1), ...
                params.length(swing_num), params.apex, 0, 0);
    cost = dot(weights.*(fk_pos - des_swing_pos), weights.*(fk_pos - des_swing_pos));
end

function cost = HeadTracking(x, target, weight, robot)
    fk_pos = ForwardKinematicsCasadi(robot, x, 3, [0; robot.torso_length]);
    cost = dot(weight.*(fk_pos - target), weight.*(fk_pos - target));
end

