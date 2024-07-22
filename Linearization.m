%% linearization
clear all

% Define the operating point (equilibrium point)
Xin = [0 0 0 0 0 0];  % [x_k, y_k, psi_k, u_k, v_k, r_k]
Uin = [0 0 0];  % [tau_x_k, tau_y_k, tau_psi_k]
dt = 0.1;  % Time step

% A, B, C, D identifcation
[A, B] = linearize_model(Xin, Uin, dt)
C = eye(6);
D = zeros(6, 3);

%ss system
Ship_sys=ss(A,B,C,D);

%transfer function
ship_tf = tf(Ship_sys);

% controllability
Co = ctrb(A,B);
Controllability = 1-(length(A)-rank(Co))

% Stability
stability=isstable(Ship_sys)

% Poles
P = pole(Ship_sys)


%linearization function
function [A, B] = linearize_model(Xin, Uin, dt)
    % Define numerical perturbation
    delta = 1e-5;
    
    % Initialize A and B matrices
    n = length(Xin);
    m = length(Uin);
    A = zeros(n, n);
    B = zeros(n, m);
    
    % Compute A matrix (Jacobian w.r.t. state)
    for i = 1:n
        dX = zeros(n, 1);
        dX(i) = delta;
        X_plus = RK4(Xin + dX, Uin', dt);
        X_minus = RK4(Xin - dX, Uin', dt);
        A(:, i) = (X_plus - X_minus) / (2 * delta);
    end
    
    % Compute B matrix (Jacobian w.r.t. input)
    for i = 1:m
        dU = zeros(m, 1);
        dU(i) = delta;
        X_plus = RK4(Xin, Uin' + dU, dt);
        X_minus = RK4(Xin, Uin' - dU, dt);
        B(:, i) = (X_plus - X_minus) / (2 * delta);
    end
end

