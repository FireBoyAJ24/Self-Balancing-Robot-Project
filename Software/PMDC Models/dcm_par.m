% This file contains parameters for both 
% PM and Series dc motors
% Dynamic Modeling of DC Motors
% ELEC 343, Spring 2025

La = 120e-3; 
Ra = 7; % for Series Motor this would be 
        % the combined resistance
J = 2.12e-6; 
Dm = 6.04e-6;
Kv = 1.41e-2;  
Laf = 39.5e-3;
Va = 6;
Tm = 3.53e-3;
Ta = La/Ra;

% If you what to implement the State-Space 
% for the PM dc motor
% dx = A*x + B*u
%  y = C*x + D*u

A = [-Ra/La,-Kv/La;
      Kv/J,-Dm/J];
B = [1/La,0;
     0,-1/J];
C = [Kv,0;
    eye(2)]; 
D = zeros(3,2);

% Calculate the estimated starting currents
Ist_ser = Va/Ra, Te_ser = Laf*Ist_ser^2
Ist_pm = Va/Ra, Te_pm = Kv*Ist_pm
