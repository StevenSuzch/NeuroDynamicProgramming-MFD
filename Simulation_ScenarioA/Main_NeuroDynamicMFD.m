clc;clear;close all;

%% Configuration
%----------------------------------------------------------------------------------------
% A script written for running MFD system with NeuroDynamic Controller
% Version: Set-point control (Regulation problem)
%
% by: Steven Su
% Department of Architecture and Civil Engineering (ACE)
% City University of HongKong
% revised: Mar 2019
%
% Reference: (1) Neuro-dynamic programming for optimal control of macroscopic 
%                fundamental diagram systems, 
%                Transportation Research Part C: Emerging Technologies,


% This is the 'MAIN' program


tic
%% Control Settings
global Wo;
global A;
global R Q;
global Nequa_R1 Nequa_R2
global q
global neq1 neq2 neq3 neq4 
global ueq1 ueq2

% Setting A: Equilibrium point for two regions.  Accumulation -(veh)

Nequa_R1=3000;  % Region 1
Nequa_R2=3000;  % Region 2

% Example A-1,2,3,5: Nequa_R1=Nequa_R2=3000;


% Setting B: Initial state for two regions.  Accumulation -(veh)
Nzero1=3100; % Initial accumulation for Region 1
Nzero2=1800; % Initial accumulation for Region 2

% Intial states: Example A-1 [3100, 1800]; Example A-2 [4000, 2000]; 
%                Example A-3 [4000, 1800]; Example A-5 [4000, 1800]; 

% Setting C: Constant demand for four directions. Flow - (veh/s)
q=[1.6 1.6 1.6 1.6]; %[q11,q12,q21,q22]

% Demand profiles:  Example A-1,2,3, q=[1.6 1.6 1.6 1.6];
%                   Example A-5      q=[1.58 1.56 1.54 1.52];

% Setting D: Control time span.  Time - (Sec)
ti=0;       % Time to start
tf=36000;   % Time to Finish
tspan=[ti tf];

% Setting E: Weighted matrix for Value function  Equa(24)
R = 1*[1 0;0 1];  % Weighted matrix of uRu
Q=10^-2 *[1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1]; % Weighted matrix of xKx;

% Setting F: Monte-Carlo integration sampling time and Iteration Time
SampleTime = 2*10^4;
IterTime=20;

%% System Configuration

% (1): Calculate MFD G function for two equilibrium point;
g1 = Calculate_MFDGfun(Nequa_R1);
g2 = Calculate_MFDGfun(Nequa_R2);

% (2): Calculate Equilibrium accumulation for four directions; Equa(A.1)
neq1 = Nequa_R1*(q(1)+q(3))/g1;  
neq2 = Nequa_R1-neq1;             
neq4 = Nequa_R2*(q(2)+q(4))/g2;   
neq3 = Nequa_R2-neq4;             

neqArray= [neq1 neq2 neq3 neq4];

% (3): Calculate Equilibrium control control; Equa(A.2)
ueq1 = q(2)/(g1-q(1)-q(3));  
ueq2 = q(3)/(g2-q(2)-q(4));   

ueqArray= [ueq1 ueq2];

%(4): Calculate Control boundary;   0< utilde+ubar <1 ;
% Boundary for saturated controller

A=0.5;

% (5): Calculate initial state for four directions nij(0);  endogenous:exogenous=3:7
nzero1 = 0.3*Nzero1;  % n11
nzero2 = 0.7*Nzero1;  % n12
nzero3 = 0.7*Nzero2;  % n21
nzero4= 0.3*Nzero2;  % n22


% (6): Calculate initial state for four directions nij_tilde(0);
Ntilde_initial = [nzero1-neqArray(1) nzero2-neqArray(2) nzero3-neqArray(3) nzero4-neqArray(4)];

%% Calculate initial stable control policy

 [ Wo_Initial ] = Calculate_InitialControl(neqArray,ueqArray, SampleTime);
%% Calculate Policy Iteration
[ W_List,Wo ,time] = Calculate_PolicyIteration( neqArray,ueqArray,IterTime,SampleTime,Wo_Initial );

%% Calculate ODE
tic
%  [t,x]= ode23('filelin3x3',tspan,Ntilde_initial,odeset('MaxStep',1e-1));
options=odeset('RelTol',1e-6,'AbsTol',1e-6);
[t,x]=ode23('Calculate_ODE',[0:1:tf],Ntilde_initial,options);


% To remap the states of four directions and the control inputs
for i=1:length(x) 
    
    x1=x(i,1);    x2=x(i,2);    x3=x(i,3);x4=x(i,4);
    [ dPHI ] = Calculate_dPHI( x1,x2,x3,x4);
  
    g = Calculate_GBfun(x2+neq2,x3+neq3,x1+x2+Nequa_R1,x3+x4+Nequa_R2);
    

    uu = -0.5*inv(R)*g'*dPHI'*Wo;
    utilde = Calculate_SaturatedOperator( uu(1), uu(2) )  ; 
    U(i,:)=[utilde(1),utilde(2)];
    
    % Calculate onestep cost;
    [Cost ] = Calculate_OnestepCost( x1,x2,x3,x4,U(i,1),U(i,2));
    y(i)=Cost;
    
end

toc
%% Plot
figure(1);
hold on;
grid on
AA=plot(t,x(:,1),'r-');B=plot(t,x(:,2),'b-');C=plot(t,x(:,3),'g-');D=plot(t,x(:,4),'m-');
xlabel('Time[sec]');ylabel('System states');legend('ntu11','ntu12','ntu21','ntu22');
title('State errors of the sub-region dynamics');
set(AA,'LineWidth',1.5);set(B,'LineWidth',1.5);set(C,'LineWidth',1.5);set(D,'LineWidth',1.5);

figure(2);
hold on;
grid on
E=plot(t,x(:,1)+neq1,'r-');F=plot(t,x(:,2)+neq2,'b');G=plot(t,x(:,3)+neq3,'g-');H=plot(t,x(:,4)+neq4,'m-');
xlabel('Time[sec]');ylabel('Number of vehicles[veh]');legend('n11','n12','n21','n22');
title('Four directions accumulations trajectory for the nearly optimal control law ');
set(E,'LineWidth',1.5);set(F,'LineWidth',1.5);set(G,'LineWidth',1.5);set(H,'LineWidth',1.5);

figure(3);
hold on;
grid on
I=plot(t,x(:,1)+x(:,2)+neq1+neq2,'r-');J=plot(t,x(:,3)+x(:,4)+neq3+neq4,'b-');
title('Accumulation trajectory of two regions for the nearly optimal control law');
xlabel('Time[sec]');ylabel('Number of vehicles[veh]');legend('n1','n2');
set(I,'LineWidth',1.5);set(J,'LineWidth',1.5);


figure(4);
hold on;
plot(t,U(:,1)+ueq1,'r-');plot(t,U(:,2)+ueq2,'b:');
xlabel('Time[sec]');ylabel('Control input u(t)');legend('u1','u2');
%title('LQR Control Signal');
%title('The Initial Stabilizing Control: LQR Control Signal with Bounds');
title('Nearly optimal control law of u(t) with input constraints');


