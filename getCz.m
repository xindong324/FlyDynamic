function Cz = getCz(alpha0,beta0,delte0)
A = [0.77 0.241 -0.1 -0.416 -0.731 -1.053 -1.366 -1.646 -1.917 -2.12 -2.248 -2.229
];
alpha = -10:5:45;
A0 = interp1(alpha,A,alpha0,'PCHIP');
Cz=A0*(1-(beta0/57.3)^2)-0.19*(delte0/25.0);
