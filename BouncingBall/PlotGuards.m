function PlotGuards(GuardParams)
hold on;
    for i = 1:2
        plot(GuardParams.G{i}(:, 1), GuardParams.G{i}(:, 2), GuardParams.Color(i), "LineWidth", 2)
    end

    CircleAnimation(GuardParams.G{3}(1), GuardParams.G{3}(2), GuardParams.G{3}(3));
hold off;
end