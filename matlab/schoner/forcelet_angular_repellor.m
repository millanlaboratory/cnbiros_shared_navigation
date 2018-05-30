function dphidt = forcelet_angular_repellor(phi, psi, robst, Robst, Dvehicle)
% dphidt = forcelet_angular_repellor(phi, psi, robst, Robst, Dvehicle)
%
% Function to get the forcelet dphidt, given the current orientation of the
% robot (phi), the orientation and the distance of the obstacle (psi and
% robst), the size of the obstacle and size of the vehicle.

    dpsitot = forcelet_angular_repellor_subtended_angle(robst, Robst, Dvehicle);
    %dpsitot = 1.0;
    dphidt = (phi - psi).*(1./dpsitot).*exp(1 - abs( (phi - psi)./(dpsitot) ));
end