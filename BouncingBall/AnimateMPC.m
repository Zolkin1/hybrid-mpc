function AnimateMPC(t, x, BallParams, GuardParams)
figure;

pause;
for i = 1:length(t)
    clf;
    PlotGuards(GuardParams);

    CircleAnimation(x(1, i), x(2, i), BallParams.r);
    axis equal
    drawnow limitrate;
    pause(0.01);
end
end

