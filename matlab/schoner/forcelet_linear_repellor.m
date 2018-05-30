function dxdt = forcelet_linear_repellor(x, robs, Lvehicle)
% dxdt = forcelet_linear_repellor(x, robs, mindistance)
%
% Function to get the forcelet of the repellor given the current position
% (x) and the current distance of the obstacle (robs) and the minimum
% distance of the obstacle


    %dxdt = (x - robs).*exp(-(x-robs));
    
%     dxdt = sin( (0.5)*( (x - robs - minposition).*(pi)./(minposition) )).*exp(1 - abs( (x - robs - minposition)./10));
    
    dxdt = (x - robs + Lvehicle).*exp(1 - abs( (x - robs + Lvehicle) ));
end