clear all

% Parameters
dt = 0.1;  % Sample time
N = 5;  % Prediction horizon

% Initial state and target state
X0 = [0; 0; 0; 0; 0; 0]';
X_target = [4; 4; 1.57; 0; 0; 0]';

% Initial guess for control inputs
U0 = zeros(3, N);
X_history=X0;
U_history=U0(:,1);

% Main iteration loop
for j = 1:2000

% Objective function
objective = @(U) nmpcObjective(U, X0, X_target, N, dt);

% Constraints
lb = -50 * ones(3, N);  % Min 20
ub = 50 * ones(3, N);   % Max 20

% Options for fmincon
options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp');

% Solve the optimization problem
U_opt = fmincon(objective, U0, [], [], [], [], lb, ub, [], options);

% Extract the optimized control inputs
U_opt = reshape(U_opt, [3, N]);

% Simulation
X = RK4(X0, U_opt(:, 1), dt);

% Update currunt position and input
X0=X;
U0=[U_opt(:,2:N) U_opt(:, N)];

% Save data
X_history = [X_history; X];
U_history = [U_history U0(:,1)];

end

% Plot the results
time = 0:dt:(j*dt);
figure;
subplot(6,1,1); plot(time, X_history(:,1)'); title('NMPC Controlled States'); ylabel('x [m]');h = gca; h.XAxis.Visible = 'off'; grid on
subplot(6,1,2); plot(time, X_history(:,2)');  ylabel('y [m]');h = gca; h.XAxis.Visible = 'off'; grid on
subplot(6,1,3); plot(time, X_history(:,3)');  ylabel('ψ [rad]');h = gca; h.XAxis.Visible = 'off'; grid on
subplot(6,1,4); plot(time, X_history(:,4)');  ylabel('u [m/s]');h = gca; h.XAxis.Visible = 'off'; grid on
subplot(6,1,5); plot(time, X_history(:,5)');  ylabel('v [m/s]');h = gca; h.XAxis.Visible = 'off'; grid on
subplot(6,1,6); plot(time, X_history(:,6)'); xlabel('Time [s]'); ylabel('r [rad/s]'); grid on


% Plot controlled input
figure;
subplot(3,1,1); plot(time, U_history(1,:));title('NMPC Controlled Inputs'); ylabel('Tx [N]');h = gca; h.XAxis.Visible = 'off'; grid on
subplot(3,1,2); plot(time, U_history(2,:));ylabel('Ty [N]');h = gca; h.XAxis.Visible = 'off'; grid on
subplot(3,1,3); plot(time, U_history(3,:)); xlabel('Time [s]'); ylabel('Tpsi [Nm]'); grid on





function J = nmpcObjective(U, X0, X_target, N, dt)
    U = reshape(U, [3, N]);
    X = X0;
    J = 0;
    for k = 1:N
        X = RK4(X, U(:, k), dt);
        J = J + norm(X - X_target)^2;  % Quadratic cost function
    end
end