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
        circle(x(1), x(2), BallParams.r);
        axis equal
        drawnow limitrate;
        pause(0.01);
    end
end
end

function PlotGuards(GuardParams)
hold on;
    for i = 1:size(GuardParams.G, 1)
        plot(GuardParams.G(i, :, 1), GuardParams.G(i, :, 2), GuardParams.Color(i), "LineWidth", 2)
    end
hold off;
end

function h = circle(x,y,r)
    hold on
    th = 0:pi/50:2*pi;
    xunit = r * cos(th) + x;
    yunit = r * sin(th) + y;
    h = plot(xunit, yunit);
    hold off
end
