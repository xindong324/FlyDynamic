function root = FindRT(a,b,T0,eps)
kg2slug = 0.068522*3.2808399;
root = (a+b)/2;
h = 10000;
v = 190;
T1= T0;%换算为slug

[rou g va] = getRou(h);

mach = v/va;
%alpha 在5到6之间

while(1)
    T = getT(mach,h,root);
    if(abs(T-T1)<=eps)
        break;
    else if((T-T1)<0)
            a = root;
            root = (a+b)/2;
            deltt = getT(mach,h,root);
        else
            b = root;
            root = (a+b)/2;
            deltt = getT(mach,h,root);
        end
    end
end