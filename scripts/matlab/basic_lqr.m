%% Author: David Akre
% Simulation for quadrotor parameterized by IRIS parameters
clear; clear all; clear all global;
I = [0.0348 0 0; 0 0.0459 0; 0 0 0.0977];
m = 1.75;
g = 9.81;

% x = [x vx y vy z vy phi theta psi dphi dtheta dpsi]
A = [
    0 1 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 g 0 0 0;
    0 0 0 1 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 -g 0 0 0 0 0;
    0 0 0 0 0 1 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 1 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 1 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 1;
    0 0 0 0 0 0 0 0 0 0 0 0;
];
B = [
    0 0 0 0;
    0 0 0 0;
    0 0 0 0;
    0 0 0 0;
    0 0 0 0;
    1/m 0 0 0;
    0 0 0 0;
    0 1/I(1,1) 0 0;
    0 0 0 0;
    0 0 1/I(2,2) 0;
    0 0 0 0;
    0 0 0 1/I(3,3)
];

% Set C to be fully observable
C = eye(size(A, 1));

% No feed through
D = zeros(size(B));

% Check if the system is controllable via Caley Hamilton Theorem
% C = [B AB A2 B ... ], rank == # states
Co = ctrb(A, B);
assert(rank(Co) == size(A, 1))

G = ss(A, B, C, D);

R = eye(size(B,2));
Q = eye(size(A, 1));
Q(1, 1) = 5;
Q(2, 2) = 5;
Q(3, 3) = 10;
Q(7, 7) = 10;
Q(9, 9) = 10;
Q(11, 11) = 5;
K = lqr(G, Q, R);
disp('Controller')
formatted_K = sprintf('%d, ', round(K, 3));
disp(formatted_K)

% Verify CL system is stable 
G_cl = A - B*K
assert(rank(G_cl) == size(A, 1))
assert(all(eig(G_cl) < 0))
disp('CL Eigenvalues')
disp(eig(G_cl))
