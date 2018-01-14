function [Cnp Cnr] = getCnpr(alpha0)
nr = [-0.38 -0.363 -0.378 -0.386 -0.37 -0.453 -0.55 -0.582 -0.595 -0.637 -1.02 -0.804];
np = [0.061 0.052 0.052 -0.012 -0.013 -0.024 0.05 0.15 0.13 0.158 0.24 0.15];
alpha = -10:5:45;
Cnp = interp1(alpha,np,alpha0,'PCHIP');
Cnr = interp1(alpha,nr,alpha0,'PCHIP');