function [root delte]= FindRoot(a,b,eps)
root = (a+b)/2;
h = 10000;
v = 190;
S = 27.87;
m = 9298.643585;
[rou g va] = getRou(h);
W = m*g;
%alpha ��5��6֮��
delte = Zfun(root,rou,v,S,W);

while(1)
    Cm = getCm(root,delte);
    if(abs(Cm)<=eps)
        break;
    else if(Cm<0)
            a = root;
            root = (a+b)/2;
            delte = Zfun(root,rou,v,S,W);
        else
            b = root;
            root = (a+b)/2;
            delte = Zfun(root,rou,v,S,W);
        end
    end
end