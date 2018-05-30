function dxdt = forcelet_linear_attractor_forward(x, maxposition)

    dxdt = -(x - maxposition).*exp(1 - abs( (x - maxposition) ));

end