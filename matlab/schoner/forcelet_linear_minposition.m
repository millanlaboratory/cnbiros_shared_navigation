function dxdt = forcelet_linear_minposition(x, minposition)
% dxdt = forcelet_linear_minposition(x, minposition)
%
% Function to get the forcelet dxdt related to the minimum position with
% respect to an obstacle, given the current position (x) and obstacle
% distance (robst)

    dxdt = -sin( (0.5)*(x - minposition).*(pi)./(minposition) ).*exp(1 - abs(x-minposition)/0.5);

end