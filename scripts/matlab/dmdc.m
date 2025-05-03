%% Author: David Akre
%% DMDc from simulation data
clear; clear global; clear all; clc;
set(0, 'DefaultFigureRenderer', 'painters'); 

run /home/dakre/Downloads/cvx/cvx_startup.m

function sys = DMDc(states, inputs, dt)
    % Transpose data to be in shape: (state_dim x time_steps) for concatenation
    X = states(1:end-1, :)';
    X_shifted = states(2:end, :)';
    U = inputs(1:end-1, :)';

    % Construct Omega matrix (concatenate states and inputs)
    Omega = [X; U];

    % Perform SVD
    [U_svd, S_svd, V_svd] = svd(Omega, 'econ');

    % Separate U_svd into state and control parts
    n = size(X, 1);
    U_s = U_svd(1:n, :);
    U_c = U_svd(n+1:end, :);

    % Separate U_svd into state and control parts
    n = size(X, 1);
    U_s = U_svd(1:n, :);
    U_c = U_svd(n+1:end, :);

    % Compute Ad and Bd
    Ad = X_shifted * V_svd * pinv(S_svd) * U_s';
    Bd = X_shifted * V_svd * pinv(S_svd) * U_c';

    % Return discrete-time state-space system
    sys = ss(Ad, Bd, eye(n), zeros(n, size(Bd,2)), dt);
end

function sys = DMDcvx(states, inputs, dt)
    ns = size(states,2);
    ni = size(inputs,2);

    X = states(1:end-1, :)';
    X_shifted = states(2:end, :)';
    U = inputs(1:end-1, :)';

    lambda = 0.5;
        
    cvx_solver mosek
    
    cvx_begin sdp
        variable A(ns, ns)
        variable B(ns, ni)

        delta = X_shifted - A*X - B*U;

        B_pen = lambda * norm(B, 1);

        minimize(norm(delta, 'fro') + B_pen)
        A(1,3) == 9.81;
        A(3,2) == 1;
        A(3,1) == 0;
        A(2,3) == 0;
        A(3,3) == 0;
        B(3,1) == 0;
    cvx_end
    C = [0 1 0; 0 0 1];
    D = zeros(size(C,1),ni);
    sys = ss(A, B, C, D, dt);
end

function v_body = ned_to_body_velocity(v_ned, roll, pitch, yaw)
    cr = cos(roll); sr = sin(roll);
    cp = cos(pitch); sp = sin(pitch);
    cy = cos(yaw); sy = sin(yaw);

    % Rotation matrix (ZYX Euler: yaw->pitch->roll)
    R_bf = [...
        cp*cy,             cp*sy,            -sp;
        sr*sp*cy - cr*sy,  sr*sp*sy + cr*cy,  sr*cp;
        cr*sp*cy + sr*sy,  cr*sp*sy - sr*cy,  cr*cp];

    % Transform velocity
    v_body = R_bf * v_ned;
end

pos = readtable('posn.csv');
imu = readtable('imun.csv');
inputs = readtable('motr.csv');
inputs(:, {'timestamp' }) = [];

states = innerjoin(pos, imu, 'Keys', 'TimeUS');
states(:, {'timestamp_pos' }) = [];
states(:, {'timestamp_imu' }) = [];

frame = innerjoin(states, inputs, 'Keys', 'TimeUS');
frame(frame.Th0 == 0, :) = [];
frame(frame.TimeUS > 100*1e6, :) = [];
frame(frame.TimeUS < 60*1e6, :) = [];
time = frame.TimeUS;
frame(:, {'TimeUS'}) = [];
order = {'Th0', 'UR', 'UP', 'UY', 'X', 'U', 'Y', 'V', 'Z', 'W', 'Roll', 'P', 'Pitch', 'Q', 'Yaw', 'R'};
frame = frame(:, order);

% lat vel (v) + roll rate (p) + roll angle (phi)
% A 3x3 B 3x1 delta lateral (set roll)

for t = 1:length(frame.U)
    v_fixed = [frame.U; frame.V; frame.W]';
    v_body = ned_to_body_velocity(v_fixed(:,t), frame.Roll(t), frame.Pitch(t), frame.Yaw(t));
    frame.U(t) = v_body(1);
    frame.V(t) = v_body(2);
    frame.W(t) = v_body(3);
end
frame(:, {'Th0', 'UP', 'UY', 'X', 'U', 'Y', 'Z', 'W', 'Pitch', 'Q', 'Yaw', 'R'}) = [];
fr = frame(:, {'UR','V', 'P', 'Roll'});

frame = table2array(fr);

n_inputs = 1;
n_states = 3;
inputs = frame(:, 1:n_inputs);
states = frame(:, n_inputs+1:end);
assert(size(states, 2) == n_states);
assert(size(inputs, 2) == n_inputs);

%v_bias = abs(mean(states(:,1)));
%states(:,1) = states(:,1) - v_bias;

dt = mean(diff(time)) * 1e-6;

mx = [max(states(:, 1)); max(states(:, 2)); max(states(:, 3))];
states(:, 1) = states(:, 1) / mx(1);
states(:, 2) = states(:, 2) / mx(2);
states(:, 3) = states(:, 3) / mx(3);
disp(mx)

G = DMDcvx(states, inputs, dt);
%G = DMDc(states, inputs, dt);

eig_A = eig(G.A);
disp('Eigenvalues of OL system')
disp(sort(real(eig_A)));

Wc = dlyap(G.A, G.B*G.B');
assert(rank(Wc) == n_states);
[v_Wc, d_Wc] = eig(Wc);
sorted_eig_Wc = sort(real(diag(d_Wc)));
for i = 1:length(sorted_eig_Wc)
    if sorted_eig_Wc(i) <= 0.01
        disp(['Gram Eigenvalue ', num2str(sorted_eig_Wc(i))])
        disp('Gram Eigenvector')
        disp(v_Wc(:, i));
    end
end

X_dmdc_sim = lsim(G, inputs, []);
time_s = time / 1e6;
figure;
n_obs = size(G.C, 1);
for i = 1:n_obs
    subplot(n_obs,1,i);
    plot(time_s, states(:, i)*mx(i), 'b', 'LineWidth', 1.5); hold on;
    plot(time_s, X_dmdc_sim(:, i)*mx(i), 'r--', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel(['State x_' num2str(i)]); 
    legend('Measured', 'DMDc Simulated');
    grid on;
end
title('State Trajectory Comparison');

figure;
for i = 1:n_obs
    subplot(n_obs,1,i);
    delta = mx(i)*(states(:, i) - X_dmdc_sim(:, i));
    plot(time_s, delta, 'b', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel(['Delta State x_' num2str(i)]); 
    grid on;
end
title('Delta (Measured - DMD Sim)');
