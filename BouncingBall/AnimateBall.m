function AnimateBall(sol, BallParams, GuardParams)
sim = figure;
figure(sim);

pause;
for j = 1:length(sol)
    t = linspace(sol(j).x(1), sol(j).x(end), ceil((sol(j).x(end)-sol(j).x(1))*100));
    for i = 1:length(t)
        clf;
        PlotGuards(GuardParams);
    
        x = deval(sol(j), t(i));
        CircleAnimation(x(1), x(2), BallParams.r);
        axis equal
        drawnow limitrate;
        pause(0.01);
    end
end
end

