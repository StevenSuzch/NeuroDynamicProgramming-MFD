function [ utilde ] = Calculate_SaturatedOperator( utilde1, utilde2  )


%% Configuration

% This is to calculated the bounded control using the saturation operator

% Equa(18)---Fig(5)

% By: Steven SU

% Parameter settings of Equa(18):
% A=0.5, a=1, b=-0.5, c=0.5

%% Saturation operator
global A;
global  ueq1 ueq2


u = [A*tanh(1/A*(utilde1+ueq1-0.5))+0.5;
    A*tanh(1/A*(utilde2+ueq2-0.5))+0.5];

if u(1)>=(1-1e-14)
    u(1)=(1-1e-14);
elseif  u(1)<=0
    u(1)=0;
end

if u(2)>=(1-1e-14)
    u(2)=(1-1e-14);
elseif u(2)<=0
    u(2)=0;
end
utilde=[u(1)-ueq1;u(2)-ueq2];



end

