function [ W_List,Wo,time ] = Calculate_PolicyIteration( neqArray,ueqArray,IterTime,SimuTime,Wo_Initial )

%%  Configuration

% This is to conduct the policy iteration to train the NN approximator

% By: Steven SU


%%

global R Q;

global Nequa_R1 Nequa_R2

global q

global neq1 neq2 neq3 neq4 
global ueq1 ueq2

W_List=[];      % To record Wo for each iteration.

Wo=Wo_Initial;  % Initial weighted parameter from initial controller
 
for n=1:IterTime
    index=0;
    tic
    for i=1:SimuTime
        
        %%% Step 1. Generating random sample with Monte-Carlo for trainning data;
        
        x=rand(1,4)*4000-2000;
        x1=x(1);x2=x(2);x3=x(3);x4=x(4);
        
        Ntilde1=x1+x2;
        Ntilde2=x3+x4;
        
        %%% Step 2.  Calculate dPhi matrix;
        
        [ dPHI ] = Calculate_dPHI( x1,x2,x3,x4);
        
                
        %%% Step 3.  Calculate control nonlinear dynamical affine system of ntilde
        f1 = Calculate_Afun(x(1)+neq1,x(4)+neq4,x(1)+x(2)+Nequa_R1,x(3)+x(4)+Nequa_R2);
        g =Calculate_GBfun(x(2)+neq2,x(3)+neq3,x(1)+x(2)+Nequa_R1,x(3)+x(4)+Nequa_R2);
        f2 = Calculate_delta(neqArray,ueqArray,x+neqArray); 
        
        %%% Step4. Calculate constrainted control
        
        % (1) Calculate optimal control
        U = -0.5*inv(R)*g'*dPHI'*Wo ;    % Thesis Equa (3-9)
        
        % (2) Calculate saturated operator;
    utilde = Calculate_SaturatedOperator( U(1), U(2) )  ; 
       

        %%% Step 5: Calculate integration
        index=index+1 ;
        
        %(1): Calculate dPHI*(f+gu)
        Z= dPHI*(f1+f2)+dPHI*g*utilde;
        
        X(index,:)=Z';   
        
        % (2): Calculate Cost function 
            [Cost ] = Calculate_OnestepCost( x1,x2,x3,x4,utilde(1),utilde(2));
        y(index,1)  =Cost;
        

        
    end

    A=X'*X; 

    
    B=X'*y; 
    
    AA=pinv(A); % Invert first part
    
    W=AA*B;  % Calculate weight
    
    W_List=[W_List W];
    
    Wo=W;
    
    time(n)=toc;
end
Wo   % The final weighted parameter 



end

