function [ u1, u2 ] = Calculate_HaddadControl( Ntilde1 ,Ntilde2)
%% Configuration
% This is to calculate control input by controller in 
% (Haddad 2015) Robust constrained control of uncer / Part C

%  input:    1. Ntilde1: Extra N for region1 , Ntilde1=ntilde1+ntilde2;
%            2. Ntilde2: Extra N for region2 , Ntilde1=ntilde3+ntilde4;  

%  Output:   1.u1
%            2.u2

% By: Steven SU

%%
global A

if [1 0;0 1;-1 0;0 -1; 0.7071 -0.7071;-0.7071 0.7071;]*[Ntilde1;Ntilde2]<=[300;300;1500;1500;83.2;83.2]
    u1=[0.0042 -0.0042]*[Ntilde1;Ntilde2];
    u2=-u1;
elseif  [1 0;0 1; 0.7071 -0.7071;-0.7071 0.7071;]*[Ntilde1;Ntilde2]<=[300;1500;1211.49;-83.2]
    u1=10^-4*[-0.4213 0.4213]*[Ntilde1;Ntilde2]+0.5;
    u2=-u1;
elseif  [0 1; -1 0;-0.7071 0.7071;0.7071 -0.7071;]*[Ntilde1;Ntilde2]<=[300;1500;1211.49;-83.2]
    u1=10^-4*[-0.4213 0.4213]*[Ntilde1;Ntilde2]-0.5;
    u2=-u1;
elseif  [1 0;0 -1;-0.7071 0.7071; 0.7071 -0.7071;0.9729 -0.2314]*[Ntilde1;Ntilde2]<=[300;1500;-1211.49;1261.14 ;625.86]
    u1=10^-4*[-0.4202 0.4202]*[Ntilde1;Ntilde2]+0.5;
    u2=-u1;
elseif  [0 1;-1 0; 0.7071 -0.7071;-0.7071 0.7071;-0.2314 0.9729 ]*[Ntilde1;Ntilde2]<=[300;1500;-1211.49;1261.14; 625.86]
    u1=10^-4*[-0.4202 0.4202]*[Ntilde1;Ntilde2]-0.5;
    u2=-u1;

else 
    % Out of boundary
            u = [A*tanh((1/A)*(10^(-4)*(-0.4213*Ntilde1+0.4213*Ntilde2)-0.5));
            A*tanh((1/A)*(1-(10^(-4)*(-0.4202*Ntilde1+0.4202*Ntilde2)-0.5)))];  % Control Policy :Equa (3) of Jack Haddad(2015);
    
        % (2): Split
        u1=u(1);u2=u(2);
end
end
