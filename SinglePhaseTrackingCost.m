function cost = SinglePhaseTrackingCost(t, x, u, dt, robot, params)
    cost = 0;
    q = x(1:robot.nq);
    v = x(robot.nq + 1: robot.nq + robot.nv);
    pos = x(11:end);

    q_target = interp1(params.t_ref, params.q_target', t)';
    v_target = interp1(params.t_ref, params.v_target', t)';
    u_target = interp1(params.t_ref, params.u_target', t)';
    pos_target = interp1(params.t_ref, params.pos_target', t)';

    cost = cost + qTracking(q, q_target, params.q_weight);
    cost = cost + vTracking(v, v_target, params.v_weight);
    cost = cost + uTracking(u, u_target, params.u_weight);
    cost = cost + posTracking(pos, pos_target, params.pos_weight);

    cost = cost * dt;
end

function cost = qTracking(q, target, weight)
    cost = dot(weight.*(q - target), weight.*(q - target));
end

function cost = vTracking(v, target, weight)
    cost = dot(weight.*(v - target), weight.*(v - target));
end

function cost = uTracking(u, target, weight)
    cost = dot(weight.*(u - target), weight.*(u - target));
end

function cost = posTracking(pos, target, weight)
    cost = dot(weight.*(pos - target), weight.*(pos - target));
end