function [ ] = MotorPDI( )
% Demo a PDI controller with the motor
% Our ODE is:
% q = mmI * diff(theta,t$2) = mmI * alpha
% => alpha = q/mmI
% Setting up a state space model:
% x = [theta, omega]
% x_dot = [omega, alpha]
% x_dot = Ax + Bu
% => A = [0 1; 0 0], u = q, B = [0; 1/mmI]
% y = Cx + Du
% y = [1 0; 0 1]x + [0]u

clear all
close all

% State space model
mmI = 1;
A = [0 1; 0 0];
B = [0; 1/mmI];
C = [1 0; 0 1];
D = [0 ; 0];
sys = ss(A,B,C,D);

% Discretize?
% This looks helpful:
% http://www.mathworks.com/help/control/ref/c2d.html#inputarg_sys

% Time vector
numTPoints = 1000;
t = linspace(0,4,numTPoints);

% Input vector
u = t;
u(1:numTPoints/2) = 1;
u(numTPoints/2:end) = -1;

% Simulate
lsim(sys,u,t)
title('Testing State Space Model')
end

