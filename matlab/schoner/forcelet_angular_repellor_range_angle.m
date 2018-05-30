 function range = forcelet_angular_repellor_range_angle(phi, psiobst, delta, robst, Robst, Dvehicle)
% range = forcelet_angular_repellor_range_angle(phi, psiobst, delta, robst, Robst, Dvehicle)
%
% Function to the get the angular range weight given the current
% orientation of the robot (phi), the angular range to consider the influence
% negligble, the orientation and the distance of the
% obstacle (phiobst and robst), the size of the obstacle (Robst) and size
% of the vehicle (Dvehicle).
    
    dpsitot = forcelet_angular_repellor_subtended_angle(robst, Robst, Dvehicle);
   
    h1 = 4./( cos(2.*dpsitot) - cos(2.*dpsitot + delta) );
    range = 0.5.*( tanh( h1.*(cos(phi - psiobst) - cos(2.*dpsitot + delta) ) ) + 1 );
    
end