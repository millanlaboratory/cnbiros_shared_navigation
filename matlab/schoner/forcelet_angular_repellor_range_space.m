function range = forcelet_angular_repellor_range_space(d0, robst, Robst, Dvehicle)
% range = forcelet_angular_repellor_range_space(d0, robst, Robst, Dvehicle)
%
% Function to get the spatial range weight given the minimal distance to have
% appreciable strength, the current distance and size of the obstalce 
% (robst, Robst) and the size of the vehicle (Dvehicle).

    range = exp( -( (robst - Robst - Dvehicle)./d0 ) );
    
end