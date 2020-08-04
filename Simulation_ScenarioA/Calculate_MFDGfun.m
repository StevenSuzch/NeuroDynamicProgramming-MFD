function G=Calculate_MFDGfun(y)

% This is to calculate the production function (G function) of MFD systme
% By: Steven SU

a1 = 1.4877e-7; a2 = -2.9815e-3; a3 = 15.0912; % Parameter of MFD system
if y<0||y>10000
    G = 0;
else G = (a1*(y^3) + a2*(y^2) + a3*y)/3600;
end