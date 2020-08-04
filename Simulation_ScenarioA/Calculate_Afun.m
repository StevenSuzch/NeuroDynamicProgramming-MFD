function A=Calculate_Afun(n1,n4,n5,n6)
G1 = Calculate_MFDGfun(n5); 
G2 = Calculate_MFDGfun(n6); 
A = [-n1/n5*G1;
     0;
     0;
     -n4/n6*G2];