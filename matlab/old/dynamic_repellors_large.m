clearvars; clc;


% 
% 
% Robs = @(phi, psi, dpsi) ((phi-psi)./dpsi).*exp(1 - abs( (phi - psi)./dpsi ));
% Wobs = @(phi, psi, dpsi, sigma, h1) 0.5.*(tanh( h1.*(cos(phi-psi) - cos(2.*dpsi + sigma))) + 1);
% Dobs = @(r, Rr, Ra, d0) exp(-((r - Rr - Ra)./d0));


Robs = @(phi, psi, dpsi) ((psi-phi)./dpsi).*exp(1 - abs( (psi - phi)./dpsi ));
Wobs = @(phi, psi, dpsi, sigma, h1) 0.5.*(tanh( h1.*(cos(psi-phi) - cos(2.*dpsi + sigma))) + 1);
Dobs = @(r, Rr, Ra, d0) exp(-((r - Rr - Ra)./d0));


nsectors = 100;
psi = -pi/2:pi/nsectors:pi/2;
% psi = 0:pi/nsectors:pi;
dpsi =0.1;
% phi_robot = pi/2.0;
phi_robot = 0.0;
sigma = 0.2;
h1 = 0.1;
Rr = 0.20;
Ra = 0.4;
d0 = 1.3;

r = 1.5;
Fobs = Robs(phi_robot, psi, dpsi) + Wobs(phi_robot, psi, dpsi, sigma, h1) + Dobs(r, Rr, Ra, d0);



plot(psi, Fobs*0.1);

