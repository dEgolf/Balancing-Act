function LQRBeamBal( )
% Gets feedback law using LQR
% The idea is to minimize a cost function over the operation of the device
% --The cost function is based on states and inputs being nonzero
% --Q tells us how much we care about each state being nonzero
% --R tells us how much we care about each input being nonzero

% We want to modify the system from:
% x_dot = Ax + Bu 
% to
% x_dot = Ax + B*(-K*x)

% Get system matrices
[A, B, C, D] =  formSystem();

% Create system
sys = ss(A,B,C,D);

Q = diag([10,1,1,1]); % How much we want each of the states to be zero
% The states are:
% position, velocity, angle, angular velocity

R = diag(1); % How much we want to minimize input - the torque

N = 0; % For simplicity

[K,~,~] = lqr(sys,Q,R,N);

% Display the resulting control law
disp('Control Law:') 
disp(K) % [-3.1888   -2.5452    6.0631    1.0045]
% We set u = -Kx
end

