function lambda = angular_influence_distance(x, strength, decay, robot_size, safe_distance)
% lambda = angular_influence_distance(x, strength, decay, robot_size, safe_distance)
%
% Function to compute the lambda: the contribution to the repellor forcelet
% dependendent on the distance of the obstacle (x), the strength and the
% spatial decay as well as the robot size and the safe distance.
% robot_size is a 4 elements vector with [front_size back_size right_size left_size]

    front_robot_size = get_robot_size(robot_size, 'front');

    lambda = strength.*exp(- ( (x - front_robot_size - safe_distance)./decay ) );


end