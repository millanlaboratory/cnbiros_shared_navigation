function sigma = angular_influence_occupancy(x, robot_size, safe_distance)
% sigma = angular_influence_occupancy(x, robot_size, safe_distance)
%
% Function to compute sigma: the occupancy influence to the repellor
% forcelet given the distance and the angle of the obstacle (x and theta)
% as well as the robot size and a safe distanxe. robot_size is a 4 element
% vector with [front_size back_size right_size left_size]

    robot_width = get_robot_size(robot_size, 'right-left');

    % Subtended sensor sector (check for x < robot_width, arg of asin > 1)
    if( x < (robot_width + safe_distance) )
        dtheta = pi/2.0;
    else
        dtheta = asin( (robot_width + safe_distance) ./ x );
    end
    
    sigma = atan( tan(dtheta/2.0) + (robot_width + safe_distance)./(robot_width + safe_distance + x) );


end