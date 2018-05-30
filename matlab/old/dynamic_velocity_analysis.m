clearvars; clc;


obstacle_distance = 0:0.05:5;       %  [m]
obs_min_distance  = 1.2;            % [m]
obs_max_distance  = 4.0;            % [m]
vel_minimum = 0.10;                 % [m/s]
vel_maximum = 0.29;                 % [m/s]
strength = 0.5;


velocity = zeros(length(obstacle_distance), 1);
for oId = 1:length(obstacle_distance)
    velocity(oId) = dynamic_velocity_linear(obstacle_distance(oId), obs_min_distance, obs_max_distance, vel_minimum, vel_maximum, strength);
end



%% Plotting

hold on;
plot(obstacle_distance, velocity);
grid on;
plot_hline(0, 'k');

plot(obstacle_distance(find(velocity == vel_maximum, 1)), vel_maximum, 'ro');
plot(obs_min_distance, vel_minimum, 'ro');
plot(obs_min_distance-0.05, -vel_minimum, 'ro');
hold off;

xlabel('Obstacle distance [m]');
ylabel('Velocity [m/s]');
title('Linear velocity profile');