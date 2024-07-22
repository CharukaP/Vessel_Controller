%% Non linear function

function[Xout]=RK4(Xin,Uin,dt)

% Initialization
% Zero Disturbance
Wave_in=zeros(3,1);V_C=zeros(1,3);

pos=Xin(1:3);
vel=Xin(4:6);

myJ1=[cos(pos(3)) -sin(pos(3)) 0; sin(pos(3)) cos(pos(3)) 0; 0 0 1];


pos1=pos;
vel1=vel;
X_in1=[vel1 pos1(3)];
[acc1]=vessel_model(X_in1,Uin,Wave_in,V_C); % acceleration function

pos2=pos+(0.5*myJ1*vel1'*dt)';
vel2=vel+0.5*acc1*dt;
X_in2=[vel2 pos2(3)];
[acc2]=vessel_model(X_in2,Uin,Wave_in,V_C);

pos3=pos+(0.5*myJ1*vel2'*dt)';
vel3=vel+0.5*acc2*dt;
X_in3=[vel3 pos3(3)];
[acc3]=vessel_model(X_in3,Uin,Wave_in,V_C);

pos4=pos+(myJ1*vel3'*dt)';
vel4=vel+acc3*dt;
X_in4=[vel4 pos4(3)];
[acc4]=vessel_model(X_in4,Uin,Wave_in,V_C);


pos_new = pos + ((dt/6.0)*myJ1*(vel1 + 2*vel2 + 2*vel3 + vel4)')';
vel_new = vel + (dt/6.0)*(acc1 + 2*acc2 + 2*acc3 + acc4);

Xout=[pos_new vel_new];

end

function[LAV_dot]=vessel_model(x,ui,Wave_in,V_C)

% 3 DOF vessel model for ship dynamic positioning. Units are in kg and meters. Thesis : Dynamic Positioning by Nonlinear
%Model Predictive Control, Åsmund Våge Fannemel, NTNU, 2008
%Let ve=J1(phy,theta,psi)vo , ve->earth-fixed, vo->body-fixed, J1->coordinate transform matrix
 % Matrix gx represents this body-fixed restoring forces (gx=(inverse of % J1)*ge), ge-earth-fixed restoring forces 
%u,v,w,p,q,r - body-fixed linear and angular velocities, %phy, theta, psi - eular angles. All angles in radians

% let ui =Fx, Fy, T represenring forces in body fixed directions and torque

%%%%%%%%%%%%%%%%%%%%%%%%%%%% Viking Ship Parameters %%%%%%%%%%%%%%%%%%%%%%

m = 1229; 
xG= 0; 
Iz=1.57765e03; 
Xu_dot= -61.45; 
Yv_dot= -1087.3; 
Yr_dot= -0.0;
Nv_dot= -0.0; 
Nr_dot= -1793.13;
Xu =-25751; %
Xu_u = 1.2774e3;
Yv = -25796; %
Yv_v = 11109;
Yv_r = 0;
Yr = 0;
Yr_v = 0;
Yr_r = 0;
Nv = 0;
Nr = -500; %
Nv_v=0;
Nr_v=0;
Nv_r=0;
Nr_r =-3.85;
 

 M_rb= [ m 0 0; 0 m m*xG; 0 m*xG Iz];
 M_A=-1*[Xu_dot 0 0; 0 Yv_dot 0; 0 0 Nr_dot]; % added mass matrix
 M = M_rb+M_A;
 M_inv_mat= inv(M); % First method  
   
   
u=x(1); v=x(2); r=x(3); psi=x(4);
LAV =[u v r]; % LAV is the matrix representing linear and angular velocities
J1=[cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];


V_ref=LAV'-J1'*V_C';
ur=V_ref(1); vr=V_ref(2); %world frame velocities
%C_RB=[ 0 0 -m*(xG*r+v); 0 0 m*u; m*(xG*r+v) -m*u 0];  
  
%C_A=[0 0 (Yv_dot*vr)+0.5*(Nv_dot+Yr_dot)*r; 0 0 -Xu_dot*ur; -((Yv_dot*vr)+0.5*(Nv_dot+Yr_dot)*r ) Xu_dot*ur 0];

D_L =-1*[Xu 0 0; 0 Yv Yr; 0 Nv Nr];

%D_NL=-1*[Xu_u*abs(ur) 0 0; 0 (Yv_v*abs(vr)+Yr_v*abs(r)) (Yv_r*abs(vr)+Yr_r*abs(r)); 0 (Nv_v*abs(vr)+Nr_v*abs(r)) (Nv_r*abs(vr)+Nr_r*abs(r))];

D=D_L;

J1_T_dot=r*[-sin(psi) cos(psi) 0; -cos(psi) -sin(psi) 0; 0 0 0];


F1= -D*V_ref + M_A*(J1_T_dot*V_C');
F1=F1/m; % normalizing based on mass of the vessel


F2=ui+Wave_in; %Wave is considered as a force
F=F1+F2;


LAV_dot=(M_inv_mat*F)';

end
