function B=Calculate_GBfun(n2,n3,n5,n6)
G1 = Calculate_MFDGfun(n5);
G2 = Calculate_MFDGfun(n6);
B = [0 n3/n6*G2;
     -n2/n5*G1 0;
     0 -n3/n6*G2;
     n2/n5*G1 0];