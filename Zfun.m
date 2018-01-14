function delte = Zfun(alpha,rou,v,s,w)
x = -10:5:45;
y = [0.77 0.241 -0.1 -0.416 -0.731 -1.053 -1.366 -1.646 -1.917 -2.12 -2.248 -2.229];
delte = interp1(x,y,alpha)+2*w*cos(alpha*pi/180)/(rou*v*v*s);
