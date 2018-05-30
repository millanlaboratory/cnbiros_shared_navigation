function dphidt = forcelet_angular_attractor(phi, psi, a)
% dphidt = forcelet_angular_attractor(phi, psi, a) 
% 
% Function to get the forcelet dphidt, given the current orientation of
% the robot (phi) and the orientation of the obstacle (psi).
% a is the the time scale of the dynamic (tau_phi = 1/a, needs to be 2pi periodic)

    dphidt = -a*sin(phi-psi);

    

end