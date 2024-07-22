% sample time
dt=0.1;

% Define the initial and target positions
x_target = 4;
y_target = 4;
psi_target = 1.5708;

% Initial conditions
x_k = 0; y_k = 0; psi_k = 0;
u_k = 0; v_k = 0; r_k = 0;

% PID gains
Kp = [-3.35; 12.9; 1];  % Proportional gains for x, y, psi
Ki = [3.34; -12.9; 24.4];  % Integral gains for x, y, psi
Kd = 0*[1; 1; 1];  % Derivative gains for x, y, psi

% Time settings
Ts = 0.1;  % Sampling time
T = 50;  % Total simulation time
time = 0:Ts:T;

% Preallocate arrays
x = zeros(1, length(time));
y = zeros(1, length(time));
psi = zeros(1, length(time));
u = zeros(1, length(time));
v = zeros(1, length(time));
r = zeros(1, length(time));
tau_x = zeros(1, length(time));
tau_y = zeros(1, length(time));
tau_psi = zeros(1, length(time));

% Initial state
x(1) = x_k; y(1) = y_k; psi(1) = psi_k;
u(1) = u_k; v(1) = v_k; r(1) = r_k;

% Initialize PID errors
e_prev = [0; 0; 0];
e_int = [0; 0; 0];

% Run simulation
for k = 1:length(time)-1
    % Current errors
    e = [x_target - x(k); y_target - y(k); psi_target - psi(k)];
    
    % PID control input
    e_int = e_int + e * Ts;
    e_der = (e - e_prev) / Ts;
    
    tau = Kp .* e + Ki .* e_int + Kd .* e_der;
    tau_x(k) = tau(1);
    tau_y(k) = tau(2);
    tau_psi(k) = tau(3);
    
    % Update previous error
    e_prev = e;
    
    % Update ship state using the dynamics function
    [x_k1, y_k1, psi_k1, u_k1, v_k1, r_k1] = ship_dynamics(x(k), y(k), psi(k), u(k), v(k), r(k), tau_x(k), tau_y(k), tau_psi(k),dt);
    
    % Store new state
    x(k+1) = x_k1;
    y(k+1) = y_k1;
    psi(k+1) = psi_k1;
    u(k+1) = u_k1;
    v(k+1) = v_k1;
    r(k+1) = r_k1;
end

% Plot results
figure;
subplot(3,1,1); plot(time, x); title('x Position'); xlabel('Time [s]'); ylabel('x [m]');
subplot(3,1,2); plot(time, y); title('y Position'); xlabel('Time [s]'); ylabel('y [m]');
subplot(3,1,3); plot(time, psi); title('ψ (Yaw)'); xlabel('Time [s]'); ylabel('ψ [rad]');



function [x_k1, y_k1, psi_k1, u_k1, v_k1, r_k1] = ship_dynamics(x_k, y_k, psi_k, u_k, v_k, r_k, tau_x_k, tau_y_k, ta_psi_k,dt)
    
    Xin=[x_k y_k psi_k u_k v_k r_k];

    T_inn=[tau_x_k; tau_y_k; ta_psi_k];

    [Xout]=RK4(Xin,T_inn,dt);

    x_k1=Xout(1);
    y_k1=Xout(2);
    psi_k1=Xout(3);
    u_k1=Xout(4);
    v_k1=Xout(5);
    r_k1=Xout(6);


end

