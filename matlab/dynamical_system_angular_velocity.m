clearvars; clc;

nsectors = 41;
heading  = 0:pi/nsectors:pi;

obs_angle       = 3*pi/4;
obs_distance    = 1.0;
obs_strength    = 0.05;
obs_decay       = 5;

robot_size = [0.93 0.65 0.375 0.375];
safe_distance = 0.1;

robot_initial_phi = pi/2;
time_limits = [0 1000];


Fobs = @(phi, theta, x, strength, decay, size, safe) forcelet_angular_repellor(phi, theta, x, strength, decay, size, safe);
                             

[t, robot_phi] = ode45(@(t, heading) Fobs(heading, obs_angle, obs_distance, obs_strength, obs_decay, robot_size, safe_distance), time_limits, robot_initial_phi);


fig1 = figure;
fig_set_position(fig1, 'Top');

subplot(1, 2, 1);
plot(heading, Fobs(heading, obs_angle, obs_distance, obs_strength, obs_decay, robot_size, safe_distance));
plot_hline(0, 'k');
plot_vline(obs_angle, 'r--', 'Obstacle angle');

subplot(1, 2, 2);
plot(t, robot_phi);
plot_hline(robot_initial_phi, 'k');
plot_hline(obs_angle, 'r--');
ylim([0 pi]);
    
