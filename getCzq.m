function Czq = getCzq(alpha0)

y = [-8.8 -25.8 -28.9 -31.4 -31.2 -30.7 -27.7 -28.2 -29 -29.8 -38.3 -35.3];
alpha = -10:5:45;
Czq = interp1(alpha,y,alpha0,'PCHIP');