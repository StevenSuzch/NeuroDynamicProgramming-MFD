function [Cost ] = Calculate_OnestepCost( x1,x2,x3,x4,utilede1,utilede2)
%% Configuration

% This is to calculate the onestep cost of each simulation interval (Equa 24)

% By: Steven SU


%%
global A R;
global Q;
global ueq1 ueq2

%% Cost of control variables
u1=utilede1+ueq1-0.5;
u2=utilede2+ueq2-0.5;

if u1>=A*(1-1e-14)
    u1=A*(1-1e-14);
elseif u1<=-A*(1-1e-14)
    u1=-A*(1-1e-14);
end

if u2>=A*(1-1e-14)
    u2=A*(1-1e-14);
elseif u2<=-A*(1-1e-14)
    u2=-A*(1-1e-14);
end

% Equa (19)
ControlCost =-2*A*R(1,1)*(u1*atanh(u1/A)+0.5*A*log(1.0-(u1/A)^2))-2*A*R(2,2)*(u2*atanh(u2/A)+0.5*A*log(1.0-(u2/A)^2));


%% State cost  Equa (20)
StateCost =-[x1 x2 x3 x4]*Q*[x1;x2;x3;x4];



%% Summation
Cost =StateCost+ControlCost;


end

