%% PID tuning

clear all

% Given system matrices form Linearization
A = [1.0000 0 0 0.0999 0 0;
     0 1.0000 0 0 0.1000 0;
     0 0 1.0000 0 0 0.1000;
     0 0 0 0.9984 0 0;
     0 0 0 0 0.9991 0;
     0 0 0 0 0 1.0000];

B = 1.0e-04 * [0.0387 0 0;
               0 0.0216 0;
               0 0 0.0148;
               0.7743 0 0;
               0 0.4315 0;
               0 0 0.2967];

C = eye(6);
Dmat = zeros(6, 3);

% ss system
sys = ss(A, B, C, Dmat);

% transfer function
dship = tf(sys);

dship.InputName = {'Tx', 'Ty', 'Tpsi'};
dship.OutputName = {'Y'};

% Tunable decoupler
D = tunableGain('Decoupler', [eye(3) eye(3)]);
D.InputName = 'e';
D.OutputName = {'Px', 'Py', 'Ppsi'};

% Tunable PID controllers
PI_x = tunablePID('PI_x', 'pi');
PI_x.InputName = 'Px';
PI_x.OutputName = 'Tx';

PI_y = tunablePID('PI_y', 'pi');
PI_y.InputName = 'Py';
PI_y.OutputName = 'Ty';

PI_psi = tunablePID('PI_psi', 'pi');
PI_psi.InputName = 'Ppsi';
PI_psi.OutputName = 'Tpsi';

% Summing junction for error calculation
sum1 = sumblk('e = r - Y', 6);

% Connecting all blocks
C0 = connect(PI_x, PI_y, PI_psi, D, sum1, {'r', 'Y'}, {'Tx', 'Ty', 'Tpsi'});

% Desired crossover frequencies (one for each loop)
wc = [0.1, 1];

% Loop tuning
[dship, C, gam, Info] = looptune(dship, C0, wc);

% Display the results
showTunable(C);

T = connect(dship,C,'r','Y');
step(T)

