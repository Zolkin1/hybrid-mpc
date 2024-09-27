function AnimateRobotGround(robot, t, q, pos, te, qe, ve, pos_e, ground)
%ANIMATEROBOT Animates the robot
%   

qd = zeros(robot.nv, 1);

figure;
title("Robot Animation")
xlabel("x (m)")
ylabel("y (m)")
pause;

offset = zeros(2,1);
e_idx = 1;

for i = 1:length(t)
    % Compute FK to get all the ends of the links
    joint_positions = zeros(2, robot.nq);
    link_positions = zeros(2, robot.nq);
    for j = 1:robot.nq
        joint_positions(:,j) = ForwardKinematics(robot, q(:, i), qd, j, [0;0]);
        link_positions(:, j) = ForwardKinematics(robot, q(:, i), qd, j, robot.link{j});
    end
    
    if i ~= 1 && e_idx <= length(te) && t(i) >= te(e_idx) && t(i-1) < te(e_idx)
        while e_idx <= length(te) && t(i) >= te(e_idx) && t(i-1) < te(e_idx)
            swing_foot_pos = ForwardKinematics(robot, qe(:, e_idx), ve(:, e_idx), robot.swing, robot.foot_r);
            swing_foot_pos
            te(e_idx)
            offset = -swing_foot_pos + offset
            e_idx = e_idx + 1;
        end
    end
    clf;

    hold on;
    %xlim([-2, 2]);
    %ylim([-2, 2]);
    axis equal;
    % for j = 1:robot.nq
    %     plot([joint_positions(1,j), link_positions(1,j)], [joint_positions(2,j), link_positions(2,j)], ...
    %         "Color", "#0072BD", "LineWidth", 2);
    %     scatter(joint_positions(1,j), joint_positions(2,j), [], [0.6350 0.0780 0.1840], "filled");
    % end
    scatter(pos(1,i), pos(2,i), [], [0.4940 0.1840 0.5560], "filled");
    PlotGround(ground);


    % Put it at the correct position
    %pos_diff = pos(:,i) - joint_positions(:, 3)
    %pos_diff(2) = 0;
    
    for j = 1:robot.nq
        joint_positions(:,j) = joint_positions(:,j) + offset;
        link_positions(:, j) = link_positions(:, j) + offset;
    end
    for j = 1:robot.nq
        plot([joint_positions(1,j), link_positions(1,j)], [joint_positions(2,j), link_positions(2,j)], ...
            "Color", "#0072BD", "LineWidth", 2);
        scatter(joint_positions(1,j), joint_positions(2,j), [], [0.6350 0.0780 0.1840], "filled");
    end

    drawnow limitrate;
    pause(.01)
end
hold off;

% Use the robot data to get the lengths of all the links

% Plot the ground

% Draw the robot

% Update the time

end

function PlotGround(ground)
% always starts at 0, plot a little before it
plot([-0.5, 0], ...
        [ground.start_height(1),...
         ground.end_height(1)], "LineWidth", 2, "Color", "k");
x0 = 0;
for i = 1:length(ground.xlength)
    plot([x0, x0 + ground.xlength(i)], ...
        [ground.start_height(i),...
         ground.end_height(i)], "LineWidth", 2, "Color", "k");
    x0 = x0 + ground.xlength(i);
end
end