function [Clp Clr] = getClpr(alpha0)
lr = [-0.126 -0.026 0.063 0.113 0.208 0.23 0.319 0.437 0.68 0.1 0.447 -0.33];
lp = [-0.36 -0.359 -0.443 -0.42 -0.383 -0.375 -0.329 -0.294 -0.23 -0.21 -0.12 -0.1];
alpha = -10:5:45;
Clp = interp1(alpha,lp,alpha0,'PCHIP');
Clr = interp1(alpha,lr,alpha0,'PCHIP');