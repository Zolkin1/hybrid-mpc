clc;
clear;
close all;
%% Simple CEM Optimization for testing

% Plot the cost function
x_plot = -10:0.1:8;
figure;
plot(x_plot, CostFcn(x_plot))
xlabel("x")
ylabel("y")

% Run CEM
nsamples = 100;
nelite = 5;
xvar = 100;
xmean = -7;
max_iters = 5;

cem_settings.nsamples = nsamples;
cem_settings.var = xvar;
cem_settings.mean = xmean;
cem_settings.nelite = nelite;
cem_settings.xsize = 1;
cem_settings.max_iters = max_iters;

for n = 1:max_iters
    % Sample
    xsample = randn(nsamples,1).*sqrt(xvar) + xmean;
    
    ysample = CostFcn(xsample);
    
    [ySorted, I] = sort(ysample);
    xSorted = xsample(I);
    ybest = ySorted(1)
    
    % Update mean
    xmean = 0;
    for i = 1:nelite
        xmean = xmean + xSorted(i);
    end
    xmean = xmean/nelite;
    
    % Update variance
    xvar = 0;
    for i = 1:nelite
        xvar = xvar + (xSorted(i) - xmean)^2;
    end
    xvar = xvar/nelite;
    
    % Plot the samples
    hold on;
    scatter(xSorted(1:nelite), ySorted(1:nelite));
    hold off;

    n
    xmean
    xvar
    mean(ySorted(1:nelite))
end

disp("CEM Function")
%[cemean, cevar, yopt] = CrossEntropyOptimization(cem_settings, @CostFcn)


disp("CEM Function2")
% Second cost function
x2_plot = -4:0.1:4;
y2_plot = -4:0.1:4;
z = zeros(length(x2_plot), length(y2_plot));

for i = 1:length(x2_plot)
    for j = 1:length(y2_plot)
        z(j, i) = CostFcn2([x2_plot(i), y2_plot(j)]);
    end
end

figure;
contour(x2_plot, y2_plot, z, 25);
xlabel("x")
ylabel("y")

cem_settings.xsize = 2;
cem_settings.mean = -2*ones(2,1);
cem_settings.var = 100*eye(2);
cem_settings.max_iters = 5;
[cemean, cevar, yopt] = CrossEntropyOptimization(cem_settings, @CostFcn2)


hold on;
scatter(cemean(1), cemean(2));
hold off;

%% Functions
function cost = CostFcn(x)
    cost = 0.1*(20*abs(x).*cos(x).^2 + exp(x) - x.^3);
end

function cost = CostFcn2(x)
    cost = x(1)^2 + x(2)^2 + 5*cos(x(1));
end