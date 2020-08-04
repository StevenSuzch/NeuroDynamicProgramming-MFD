
%% Configuration 

% This is to calculate the ODE of affine control system;

% By: Steven SU

%%
function xp=filelin3x3(t,x);



global Wo;


global R;
global Q;

global Nequa_R1
global Nequa_R2 
global q

global neq1 neq2 neq3 neq4 
global ueq1 ueq2


% Step1: Set the array of neq and ueq;
neq = [neq1 neq2 neq3 neq4];
ueq = [ueq1 ueq2];


% Step2: Set x1,x2,x3,x4 Seperately;
x1=x(1);
x2=x(2);
x3=x(3);
x4=x(4);

% Step3: Calculate dPhi;
[ dPHI ] = Calculate_dPHI( x1,x2,x3,x4);

% Step4: Calculate control affine system
f1 = Calculate_Afun(x1+neq1,x4+neq4,x1+x2+Nequa_R1,x3+x4+Nequa_R2);
f2 = Calculate_delta(neq,ueq,x'+neq);                        % Thesis Equa(2-29)
g = Calculate_GBfun(x2+neq2,x3+neq3,x1+x2+Nequa_R1,x3+x4+Nequa_R2); % Thesis Equa(2-30)

% Step5: Calculate control input;
U = -0.5*inv(R)*g'*dPHI'*Wo;

utilde = Calculate_SaturatedOperator( U(1), U(2) )  ;

U=[utilde(1);utilde(2)];



% Step7: Calculate ODE


xp(1,1)= f1(1,:)+f2(1,:)+g(1,:)*U ;
xp(2,1)= f1(2,:)+f2(2,:)+g(2,:)*U;
xp(3,1)= f1(3,:)+f2(3,:)+g(3,:)*U ;
xp(4,1)= f1(4,:)+f2(4,:)+g(4,:)*U ;
