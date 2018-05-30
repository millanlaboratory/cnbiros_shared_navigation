function [forcelet, lambda, sigma] = forcelet_angular_repellor(phi, theta, distance, strength, decay, robot_size, safe_distance)
% [forcelet, lambda, sigma] = forcelet_angular_repellor(phi, theta, distance, strength, decay, robot_size, safe_distance)
%
% Function to compute the forcelet given lambda and sigma parameters
% according to Bicho 1997.
                                                              
   lambda = angular_influence_distance(distance, strength, decay, robot_size, safe_distance);
   sigma  = angular_influence_occupancy(distance, robot_size, safe_distance);
   
  % forcelet = lambda.*(phi-theta).*(1./2.0*power(sigma, 2)).*exp( - ( power(phi - theta, 2)./(2.0.*power(sigma, 2) ) ) );
   forcelet = lambda.*(phi-theta).*(1./2.0*power(sigma, 2)).*exp( - ( power(phi - theta, 2)./(2.0.*power(sigma, 2) ) ) );
                                                              
end