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

%% Four corner test

man=1;
ship_states_final=[];
Tau_final=[];
breakpoints=[];

for man=1:5
% Set points and start point
if man==1
    disp('First Maneuver-Surge')
    X_target = [5; 0; 0; 0; 0; 0]';

elseif man==2
    disp('Second Maneuver-Sway')
    X_target = [5; 5; 0; 0; 0; 0]';

elseif man==3
    disp('Third Maneuver-Yaw')
    X_target = [5; 5; 0.78; 0; 0; 0]';

elseif man==4
    disp('Fourth Maneuver-Surge and Sway')
    X_target = [0; 5; 0.78; 0; 0; 0]';

elseif man==5
    disp('Fifth Maneuver-Surge and Sway')
    X_target = [0; 0; 0; 0; 0; 0]';
end


%% Main iteration loop
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


    % Exit criteria (within 0.01 distance and within 2.8 degree angle)
    if sqrt((X0(1)-X_target(1))^2+(X0(2)-X_target(2))^2)<0.005 && abs(X0(3)-X_target(3))<0.05;
        man=man+1;
        breakpoints=[breakpoints j];
        break
    end 

end


end %end of manuvering for loop

%% Results plot

time = 0:dt:((length(X_history)-1)*dt);
figure('Name','Position')
plot(X_history(:,2),X_history(:,1));title('Vessel Position');hold on;
x = [0, 5, 5, 0, 0];
y = [0, 0, 5, 5, 0];
plot(x, y, 'r--', 'LineWidth', 0.1);
xlabel('East') 
ylabel('Noth')

figure('Name','Input')
subplot(3,1,1); plot(time,U_history(1,:));title('Input values'); ylabel('Tx [N]')
 xline(breakpoints(1)*dt,'-r',{''});
 xline((breakpoints(1)*dt+breakpoints(2))*dt,'-r',{''});
 xline((breakpoints(1)*dt+breakpoints(2)*dt+breakpoints(3)*dt),'-r',{''});
 xline((breakpoints(1)*dt+breakpoints(2)*dt+breakpoints(3)*dt+breakpoints(4)*dt),'-r',{''});
 xline((breakpoints(1)*dt+breakpoints(2)*dt+breakpoints(3)*dt+breakpoints(4)*dt+breakpoints(5)*dt),'-r',{''});
subplot(3,1,2); plot(time,U_history(2,:)); ylabel('Ty [N]')
 xline(breakpoints(1)*dt,'-r',{''});
 xline((breakpoints(1)*dt+breakpoints(2))*dt,'-r',{''});
 xline((breakpoints(1)*dt+breakpoints(2)*dt+breakpoints(3)*dt),'-r',{''});
 xline((breakpoints(1)*dt+breakpoints(2)*dt+breakpoints(3)*dt+breakpoints(4)*dt),'-r',{''});
 xline((breakpoints(1)*dt+breakpoints(2)*dt+breakpoints(3)*dt+breakpoints(4)*dt+breakpoints(5)*dt),'-r',{''});
subplot(3,1,3); plot(time,U_history(3,:)); ylabel('Tpsi [N]'); xlabel('Time [s]')
 xline(breakpoints(1)*dt,'-r',{''});
 xline((breakpoints(1)*dt+breakpoints(2))*dt,'-r',{''});
 xline((breakpoints(1)*dt+breakpoints(2)*dt+breakpoints(3)*dt),'-r',{''});
 xline((breakpoints(1)*dt+breakpoints(2)*dt+breakpoints(3)*dt+breakpoints(4)*dt),'-r',{''});
 xline((breakpoints(1)*dt+breakpoints(2)*dt+breakpoints(3)*dt+breakpoints(4)*dt+breakpoints(5)*dt),'-r',{''});

figure('Name','Velocity')
subplot(3,1,1); plot(time,X_history(:,4));title('Velocity values'); ylabel('u [m/s]')
 xline(breakpoints(1)*dt,'-r',{''});
 xline((breakpoints(1)*dt+breakpoints(2))*dt,'-r',{''});
 xline((breakpoints(1)*dt+breakpoints(2)*dt+breakpoints(3)*dt),'-r',{''});
 xline((breakpoints(1)*dt+breakpoints(2)*dt+breakpoints(3)*dt+breakpoints(4)*dt),'-r',{''});
 xline((breakpoints(1)*dt+breakpoints(2)*dt+breakpoints(3)*dt+breakpoints(4)*dt+breakpoints(5)*dt),'-r',{''});
subplot(3,1,2); plot(time,X_history(:,5));ylabel('v [m/s]')
 xline(breakpoints(1)*dt,'-r',{''});
 xline((breakpoints(1)*dt+breakpoints(2))*dt,'-r',{''});
 xline((breakpoints(1)*dt+breakpoints(2)*dt+breakpoints(3)*dt),'-r',{''});
 xline((breakpoints(1)*dt+breakpoints(2)*dt+breakpoints(3)*dt+breakpoints(4)*dt),'-r',{''});
 xline((breakpoints(1)*dt+breakpoints(2)*dt+breakpoints(3)*dt+breakpoints(4)*dt+breakpoints(5)*dt),'-r',{''});
subplot(3,1,3); plot(time,X_history(:,6)); ylabel('r [rad/s]'); xlabel('Time [s]')
 xline(breakpoints(1)*dt,'-r',{''});
 xline((breakpoints(1)*dt+breakpoints(2))*dt,'-r',{''});
 xline((breakpoints(1)*dt+breakpoints(2)*dt+breakpoints(3)*dt),'-r',{''});
 xline((breakpoints(1)*dt+breakpoints(2)*dt+breakpoints(3)*dt+breakpoints(4)*dt),'-r',{''});
 xline((breakpoints(1)*dt+breakpoints(2)*dt+breakpoints(3)*dt+breakpoints(4)*dt+breakpoints(5)*dt),'-r',{''});

%% Cost fucntion 

function J = nmpcObjective(U, X0, X_target, N, dt)
    U = reshape(U, [3, N]);
    X = X0;
    J = 0;
    for k = 1:N
        X = RK4(X, U(:, k), dt);
        J = J + norm(X - X_target)^2;  % Quadratic cost function
    end
end