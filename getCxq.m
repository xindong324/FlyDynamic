function Cxq = getCxq(alpha0)

y = [-0.267 -0.11 0.308 1.34 2.08 2.91 2.76 2.05 1.5 1.49 1.83 1.21];
alpha = -10:5:45;
Cxq = interp1(alpha,y,alpha0,'PCHIP');