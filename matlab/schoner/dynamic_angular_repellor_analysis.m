clearvars; clc;


nsensors  = 41;
veh_angle = 0:pi/nsensors:pi;

obs_angle     = pi - pi/3;
obs_distance  = 1.0;
obs_size      = 0.1;
obs_influence_spatial = 3;
obs_influence_angular = pi/nsensors;


veh_size   = 0.75;
veh_initial_phi = pi/2.0;

time_limits = [0 10];



Fobs = @(phi, psi, robst, Robst, Dvehicle, dspatial, dangular) forcelet_angular_repellor(phi, psi, robst, Robst, Dvehicle) .* ...
                                                               forcelet_angular_repellor_range_angle(phi, psi, dangular, robst, Robst, Dvehicle) .* ...
                                                               forcelet_angular_repellor_range_space(dspatial, robst, Robst, Dvehicle) + ...
                                                               forcelet_angular_attractor(phi, psi, 0.1);

[t, phi] = ode45(@(t, veh_angle) Fobs(veh_angle, obs_angle, obs_distance, obs_size, veh_size, obs_influence_spatial, obs_influence_angular), time_limits, veh_initial_phi);
    

fig1 = figure;
fig_set_position(fig1, 'Top');
subplot(1, 2, 1);
plot(veh_angle, Fobs(veh_angle, obs_angle, obs_distance, obs_size, veh_size, obs_influence_spatial, obs_influence_angular));
plot_vline(obs_angle, 'r--', 'obstacle');
plot_hline(0, 'k');
grid on;
title('Repellor forcelet');
xlabel('phi');
ylabel('dphi/dt [rad/s]');


subplot(1, 2, 2);
plot(t, phi);
plot_hline(veh_initial_phi, 'k');
plot_hline(obs_angle, 'r--');
grid on;
ylim([-pi pi] + veh_initial_phi);
title('Vehicle orientation evolution');
xlabel('time [s]');
ylabel('phi [rad]');




