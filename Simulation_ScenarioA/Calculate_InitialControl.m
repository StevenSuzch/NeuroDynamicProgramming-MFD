function [ Wo ] = Calculate_InitialControl(neqArray,ueqArray, SampleTime)

%% Configuration

% This is to calculate a stabilizing control to initialize the policy
% iteration.

% The stalizing controller is given by:
%(Haddad 2015) Robust constrained control of uncertain / Part C'

% Output:
%     Wo: Initial weights parameters of NN;

% By: Steven SU


%%


global Nequa_R1 Nequa_R2

global neq1 neq2 neq3 neq4
global A


index=0;

for i=1:SampleTime
    
    %%% Step 1. Generating random sample with Monte-Carlo for trainning data;
    x=rand(1,4)*4000-2000;    % -2000<xi<2000
    
    x1=x(1);x2=x(2);x3=x(3);x4=x(4);
    
    % calculate regional Ntilde
    Ntilde1=x1+x2;
    Ntilde2=x3+x4;
    
    % Calculate control input using Haddad
    [ u1, u2 ] = Calculate_HaddadControl( Ntilde1 ,Ntilde2);
    
    
    %%% Step 2.  Calculate initial constrained control input with
    %%% stablizing control policy; Equa (18£©
    utilde = Calculate_SaturatedOperator( u1, u2  )  ;  %utilde
    
    %%% Step 3.  Calculate dPhi matrix;
    
    [ dPHI ] = Calculate_dPHI( x1,x2,x3,x4);
    
    
    
    %%% Step 4.  Calculate control nonlinear dynamical affine system of ntilde
    
    % Calculate F function of ntilde
    f1 = Calculate_Afun(x1+neq1,x4+neq4,x1+x2+Nequa_R1,x3+x4+Nequa_R2);
    f2 = Calculate_delta(neqArray,ueqArray,x+neqArray);                        %  Equa(A.6 F_tilde)
    
    % Calculate S function of ntilde
    g = Calculate_GBfun(x2+neq2,x3+neq3,x1+x2+Nequa_R1,x3+x4+Nequa_R2);  %  Equa(A.6 S_tilde)
    
    %%% Step 5.  Calculate integration with a Mesh;
    index=index+1;
    
    % (1) Calculate  Z= Phi(f+gu);
    Z= dPHI*(f1+f2)+dPHI*g*utilde; % Equa(7)
    X(index,:)=Z';
    
    
    
    % (2) Calculate one-step cost function  xQx+uRu;
    [Cost ] = Calculate_OnestepCost( x1,x2,x3,x4, utilde(1), utilde(2));
    y(index,1)  = Cost;
    
end

%%% Calculate weight w with Least Square method;

C=X'*X;             
B=X'*y;             
AA=pinv(C);         % Pseudo Inverse
W=AA*B;             %Equa (32)

Wo=W;


end

