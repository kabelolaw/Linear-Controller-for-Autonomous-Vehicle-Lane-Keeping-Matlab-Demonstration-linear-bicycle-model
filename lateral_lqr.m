%% Lateral Vehicle Dynamics LQR Controller Initialization
% Linear bicycle model with LQR control
clear; clc;

%% Vehicle Parameters
v = 15;         % m/s = 54 km/h (longitudinal velocity - constant)
a = 1.2;        % m - Distance from CG to front axle
b = 1.6;        % m - Distance from CG to rear axle
m = 1575;       % kg - Vehicle mass (mid-size sedan)
Iz = 2875;      % kg*m^2 - Yaw moment of inertia
Cf = 80000;     % N/rad - Front tire cornering stiffness
Cr = 80000;     % N/rad - Rear tire cornering stiffness

%% LQR Weights
q_e = 5;        % Lateral error penalty
q_psi = 10;     % Heading error penalty
q_vy = 1;       % Lateral velocity penalty
q_r = 1;        % Yaw rate penalty

Q = diag([q_e, q_psi, q_vy, q_r]);
R = 500;        % Steering effort penalty (scalar)

%% State-Space Matrices
% State vector: x = [e; psi; v_y; r]
% e: lateral position error (m)
% psi: heading angle error (rad)
% v_y: lateral velocity (m/s)
% r: yaw rate (rad/s)

% System matrix A (4x4)
A = [0,     v,      0,                          0;
     0,     0,      0,                          1;
     0,     0,      -(Cf+Cr)/(m*v),            -(a*Cf - b*Cr)/(m*v);
     0,     0,      -(a*Cf - b*Cr)/(Iz*v),     -(a^2*Cf + b^2*Cr)/(Iz*v)];

% Input matrix B (4x1)
B = [0; 
     0; 
     Cf/m; 
     a*Cf/Iz];

% Disturbance/curvature matrix E (4x1) - for future use
E = [0; -v; 0; 0];

% Output matrix
C = eye(4);     % We can measure all states
D = zeros(4,1);

%% Compute LQR Gain
[K_lqr, S, CL_poles] = lqr(A, B, Q, R);

% Display results
fprintf('\n=== LQR Controller Design ===\n');
fprintf('LQR Gain K = [%.4f, %.4f, %.4f, %.4f]\n', K_lqr);
fprintf('\nClosed-loop poles:\n');
disp(CL_poles);

% Closed-loop system
A_cl = A - B * K_lqr;

%% Initial Conditions
e_0 = 0.5;      % Initial lateral error (m)
psi_0 = 0.05;   % Initial heading error (rad)
vy_0 = 0;       % Initial lateral velocity (m/s)
r_0 = 0;        % Initial yaw rate (rad/s)

%% Saturation Limits
delta_max = 0.5;  % Maximum steering angle (rad) ≈ 28.6 degrees

%% Save workspace for Simulink
save('lateral_lqr_params.mat');
fprintf('\nParameters saved successfully!\n');
fprintf('Ready to run Simulink model.\n');
