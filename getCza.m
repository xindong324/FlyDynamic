function Cza = getCza(alpha0,delte0)

low = alpha0-1;
high = alpha0+1;
Czal = getCz(low,0,delte0);
Czah=getCz(high,0,delte0);
Cza = (Czah-Czal)/2;

% A = [0.77 0.241 -0.1 -0.416 -0.731 -1.053 -1.366 -1.646 -1.917 -2.12 -2.248 -2.229
% ];
% alpha = -10:5:45;
% %A0 = interp1(alpha,A,alpha0,'PCHIP');
% low = alpha0-1;
% high = alpha0+1;
% Czal = interp1(alpha,A,low,'PCHIP');
% Czah=interp1(alpha,A,high,'PCHIP');
% Cza = (Czah-Czal)/2;



