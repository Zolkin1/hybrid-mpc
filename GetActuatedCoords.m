function qout = GetActuatedCoords(q, robot)
    qout = zeros(robot.nj_act, 1);
    for i = 1:length(robot.act_coords)
        qout(i) = q(robot.act_coords(i));
    end
end

