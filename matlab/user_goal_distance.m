clearvars; clc;

% Logistic function
% Logistic = @(x, L, k, Q, M) L./(1+Q.*exp(-k.*(x - M) ) );

Logistic = @(x, L, k, x0) L./(1+exp(-k.*(x - x0) ) );

Slope = @(x0, MaxP, MinP, MinD) -log((MaxP/MinP) - 1)*(1/(MinD-x0));

% Parameters
MaxPosition = 2.0;
MinPosition = 0.01;
MinDistance = 0.52;
MaxDistance = 3.0;


% Support
x = 0:0.01:6;

x0 = MinDistance:0.5:4;

for i = 1:length(x0)
    cx0 = x0(i);
    cSlope = Slope(cx0, MaxPosition, MinPosition, MinDistance);

    Dgoal = Logistic(x, MaxPosition, cSlope, cx0);

    % Plot
    hold on;
    plot(x, Dgoal);
    plot(MinDistance, MinPosition, 'o');
    plot(cx0, Logistic(cx0, MaxPosition, cSlope, cx0), 'o');
    hold off;
    legend(num2str(x0));
end
xlim([0 6]);
ylim([0 MaxPosition]);


grid on;
xlabel('Dobstacle [m]');
ylabel('Dgoal [m]');
title('User goal distance position');

