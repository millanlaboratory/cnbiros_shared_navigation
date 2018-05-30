clearvars; clc;

clearvars; clc;



veh_vel = 0:0.01:10;

obs_distance          = 7.0;
obs_size              = 0.1;
obs_influence_spatial = 5.0;
veh_front_length      = 0.5;
maximum_position = 5.0;

velmin = 0;
velmax = 1;


time_limits = [0 5];


veh_initial_vel = 0;

Fobs = @(x, velmax) forcelet_linear_acceleration(x, velmax);

[t, vel] = ode45(@(t, veh_vel) Fobs(veh_vel, velmax), time_limits, veh_initial_vel);
    

subplot(1, 2, 1);
plot(veh_vel, Fobs(veh_vel, velmax));
plot_vline(velmin, 'r--', 'velmin');
plot_vline(velmax, 'r--', 'velmax');
plot_hline(0, 'k');
grid on;
title('Attractor forcelet');
xlabel('dx/dt [m/s]');
ylabel('dx^2/dt [m^2/s]');

subplot(1, 2, 2);
plot(t, vel);
plot_hline(0, 'k');
plot_hline(velmin, 'r--');
plot_hline(velmax, 'r--');
grid on;
%ylim([0 5]);
title('Vehicle velocity evolution');
xlabel('time [s]');
ylabel('dx/dt [m]');