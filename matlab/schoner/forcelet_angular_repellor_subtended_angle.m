function dpsi = forcelet_angular_repellor_subtended_angle(robst, Robst, Dvehicle)
% dpsi = forcelet_angular_repellor_subtended_angle(robst, Robst, Dvehicle)
%
% Function to get the subtended angle given the obstacle distance (robst), 
% the obstacle size (Robst) and the vehicle size (Dvehicle).

    dpsi = asin( (Robst + Dvehicle) ./ (robst + (Robst + Dvehicle)) );

end