function [Cyp Cyr] = getCypr(alpha0)
yr = [0.882 0.852 0.876 0.958 0.962 0.974 0.819 0.483 0.59 1.21 -0.493 -1.04];
yp = [-0.108 -0.108 -0.188 0.11 0.258 0.226 0.344 0.362 0.611 0.529 0.298 -2.27];
alpha = -10:5:45;
Cyp = interp1(alpha,yp,alpha0,'PCHIP');
Cyr = interp1(alpha,yr,alpha0,'PCHIP');