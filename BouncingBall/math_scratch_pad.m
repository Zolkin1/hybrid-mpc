clc;
clear;
close all;

%%
k1 = 3/2 - 0.1;
k2 = 1.5;
k3 = 2;
gamma = k3/k2;

sigeta = 3;

t0 = 1;
tf = 5;

y0 = 1;

sol = ode45(@(t, y) -(k3/k2)*y + sigeta, [t0, tf], y0);

hand_sol = @(t) y0*exp(-gamma*(t - t0)) + (sigeta/gamma)*(1 - exp(-gamma * (t - t0)));

plot(sol.x, sol.y);

tplot = linspace(t0, tf, 100);
hold on;
plot(tplot, hand_sol(tplot));
hold off;

legend(["ODE45", "Analytic"])

%%
eta = 2;
Lx = 5;
a = @(t) (sigeta/(gamma*k1))*(1 - exp(-gamma*(t - t0)));
b = @(t) (eta.*(t - t0)).*exp(Lx*(t - t0));

sigeta/k1

eta

figure;
tplot = linspace(t0, t0+1, 100);
plot(tplot, a(tplot));
hold on;
plot(tplot, b(tplot));
hold off;
legend(["a", "b"])