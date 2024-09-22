function AnimateTwoBalls(sol1, sol2, BallParams, GuardParams)
sim = figure;
figure(sim);

if sol1(end).x(end) ~= sol2(end).x(end)
    error("Solutions must have the same amount of time.")
end

pause;

t = linspace(0, sol1(end).x(end), sol1(end).x(end)*100);

for i = 1:length(t)
    clf;
    PlotGuards(GuardParams);

    x1 = GetState(sol1, t(i));
    CircleAnimation(x1(1), x1(2), BallParams.r);

    x2 = GetState(sol2, t(i));
    CircleAnimation(x2(1), x2(2), BallParams.r);

    axis equal
    drawnow limitrate;
    pause(0.01);
end

% for j = 1:length(sol1)
%     t = linspace(sol1(j).x(1), sol1(j).x(end), ceil((sol1(j).x(end)-sol1(j).x(1))*100));
%     for i = 1:length(t)
%         clf;
%         PlotGuards(GuardParams);
% 
%         x1 = deval(sol1(j), t(i));
%         circle(x1(1), x1(2), BallParams.r, [8500 0.3250 0.0980]);
% 
%         x2 = deval(sol2(j), t(i));
%         circle(x2(1), x2(2), BallParams.r, [0.4940 0.1840 0.5560]);
% 
%         axis equal
%         drawnow limitrate;
%         pause(0.01);
%     end
% end
end

function x = GetState(sol, t)
j = 1;
while sol(j).x(end) < t
    j = j + 1;
end

x = deval(sol(j), t);

end