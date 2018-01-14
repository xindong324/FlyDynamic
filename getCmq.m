function Cmq = getCmq(alpha0)

y = [-7.21 -0.54 -5.23 -5.26 -6.11 -6.64 -5.69 -6 -6.2 -6.4 -6.6 -6];
alpha = -10:5:45;
Cmq = interp1(alpha,y,alpha0,'PCHIP');