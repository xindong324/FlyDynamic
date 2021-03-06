function Cm = getCm(alpha0,elevator0)

alpha = -10:5:45;
elevator = -24:12:24;
[X Y] = meshgrid(elevator,alpha);
z = [0.205	0.081	-0.046	-0.174	-0.259;
0.168	0.077	-0.02	-0.145	-0.202;
0.186	0.107	-0.009	-0.121	-0.184;
0.196	0.11	-0.005	-0.127	-0.193;
0.213	0.11	-0.006	-0.129	-0.199;
0.251	0.141	0.01	-0.102	-0.15;
0.245	0.127	0.006	-0.097	-0.16;
0.238	0.119	-0.001	-0.113	-0.167;
0.252	0.133	0.014	-0.087	-0.104;
0.231	0.108	0	-0.084	-0.076;
0.198	0.081	-0.013	-0.069	-0.041;
0.192	0.093	0.032	-0.006	-0.005;

];
Cm = interp2(X,Y,z,elevator0,alpha0,'Cubic');
