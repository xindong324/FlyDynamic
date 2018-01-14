function [rou g a] = getRou(h)
R=6356766;
hp=h/(1+h/R);

W = 1-hp/44330.8;
T = 288.15*W;
g0 = 9.80665;
rou0 = 1.225;
rou = rou0*W^4.2559;
g =  g0*(R/(R+h))^2;
a = 20.0468*(T)^0.5;

