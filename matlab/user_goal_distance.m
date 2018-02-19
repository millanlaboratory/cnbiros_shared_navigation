clearvars; clc;

% Logistic function
Logistic = @(x, L, k, Q, M) L./(1+Q.*exp(-k.*(x - M) ) );

% Parameters
MaxDg       = 2.0;          
ZeroDg      = 0.01;
% Slope       = 5.2933;          
Slope       = 2;          
CircRadius  = 1.0;   
MidPoint    = CircRadius + 1;
ShiftX      =  (1./exp(Slope)).*((2./ZeroDg) - 1);


% Support
Dobstacle = 0:0.01:6;

Dgoal = Logistic(Dobstacle, MaxDg, Slope, ShiftX, MidPoint);

Logistic(CircRadius, MaxDg, Slope, ShiftX, MidPoint)
% Plot
hold on;
plot(Dobstacle, Dgoal);
plot(CircRadius, ZeroDg, 'o');
plot(MidPoint, Logistic(MidPoint, MaxDg, Slope, ShiftX, MidPoint), 'o');
hold off;
xlim([0 6]);
ylim([0 MaxDg]);
plot_vline(CircRadius, 'k', ['CircRadius ' num2str(Logistic(CircRadius, MaxDg, Slope, ShiftX, MidPoint))]);

grid on;
xlabel('Dobstacle [m]');
ylabel('Dgoal [m]');
title('User goal distance position');

