clearvars; clc;



veh_pos = 0:0.01:10;

obs_distance          = 7.0;
obs_size              = 0.1;
obs_influence_spatial = 5.0;
veh_front_length      = 0.5;
maximum_position = 5.0;
withobs = 0.0;


time_limits = [0 100];
veh_initial_pos = 0;



Fobs = @(x, robs, LVehicle, Robs, d0, maxpos) -withobs*forcelet_linear_repellor(x, robs, LVehicle).* ...
                            forcelet_linear_spatial_decay(robs, Robs, LVehicle, d0) + ...
                            forcelet_linear_attractor_forward(x, maxpos);

% Fobs = @(x, robs, LVehicle, Robs, d0, maxpop) forcelet_test(x, robs);

[t, pos] = ode45(@(t, veh_pos) Fobs(veh_pos, obs_distance, veh_front_length, obs_size, obs_influence_spatial, maximum_position), time_limits, veh_initial_pos);
    

fig1 = figure;
fig_set_position(fig1, 'Top');
subplot(1, 2, 1);
plot(veh_pos, Fobs(veh_pos, obs_distance, veh_front_length, obs_size, obs_influence_spatial, maximum_position));
plot_vline(obs_distance, 'r--', 'obstacle');
plot_hline(0, 'k');
grid on;
title('Repellor forcelet');
xlabel('x [m]');
ylabel('dx/dt [m/s]');


subplot(1, 2, 2);
plot(t, pos);
plot_hline(0, 'k');
plot_hline(obs_distance, 'r--');
grid on;
%ylim([0 5]);
title('Vehicle velocity evolution');
xlabel('time [s]');
ylabel('x [m]');




