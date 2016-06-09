% Create state space model for ball on beam

% x_dot = Ax + Bu
% y = Cx + Du

function [A, B, C, D] =  formSystem()
g = 9.8;        % N /kg

mBall = 0.0027;  % kg
rBall = 0.020;   % m
jBall = 2/5*mBall*rBall^2; % kg*m^2

vRamp = 0.2e-2*10e-2*40e-2; % m^3
rhoRamp = 0.7*10^3; % kg / m^3
mRamp = vRamp*rhoRamp;      % kg
lRamp = 40e-2;    % m
jRamp = 1/12*mRamp*lRamp^2; % kg*m^2

A = [0, 1, 0, 0
    0, 0, -mBall*g/(jBall/rBall^2 + mBall), 0
    0, 0, 0, 1
    -mBall*g/jRamp, 0, 0, 0];

B = [0
    0
    0
    1/jRamp];

% Assuming we observe both ball position and ramp angle
C = [1, 0, 0, 0
    0, 0, 1, 0];

D = [0
    0];