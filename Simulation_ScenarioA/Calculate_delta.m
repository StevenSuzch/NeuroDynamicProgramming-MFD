function delt=Calculate_delta(ne,ue,n)
Ge1 = Calculate_MFDGfun(ne(1)+ne(2));
Ge2 = Calculate_MFDGfun(ne(3)+ne(4));
delt = [ne(1)/(ne(1)+ne(2))*Ge1-ne(3)/(ne(3)+ne(4))*Ge2*ue(2)+n(3)/(n(3)+n(4))* Calculate_MFDGfun(n(3)+n(4))*ue(2);
        ne(2)/(ne(1)+ne(2))*Ge1*ue(1)-n(2)/(n(1)+n(2))* Calculate_MFDGfun(n(1)+n(2))*ue(1);
        ne(3)/(ne(3)+ne(4))*Ge2*ue(2)-n(3)/(n(3)+n(4))* Calculate_MFDGfun(n(3)+n(4))*ue(2);
        ne(4)/(ne(3)+ne(4))*Ge2-ne(2)/(ne(1)+ne(2))*Ge1*ue(1)+n(2)/(n(1)+n(2))* Calculate_MFDGfun(n(1)+n(2))*ue(1)];