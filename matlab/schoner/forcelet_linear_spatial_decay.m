function weight = forcelet_linear_spatial_decay(robst, Robst, Lvehicle, d0)
% weight = forcelet_linear_spatial_decay(robst, Robst, Lvehicle, d0)
%
% Function to get the spatial decay weight given the minimal distance to have
% appreciable strength (d0), the current distance and size of the obstalce 
% (robst, Robst) and the front length of the vehicle (Lvehicle).

    weight = exp( -( (robst - Robst - Lvehicle)./d0 ) );
    
end