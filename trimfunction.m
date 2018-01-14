%%---------����Z����ƽ�⣬Y������ƽ�������ƽ-------------%%
function Cm=trimfunction(r2dalpha) %ӭ�Ǻ���������ƽ����
global given_Alpha given_Elevator given_Cz given_Cm
global m g S r2d rou V0
alpha=r2dalpha/r2d;
Cz0=interp1(given_Alpha,given_Cz,r2dalpha,'spline');
elevator=25/0.19*(Cz0+m*g*cos(alpha)/(0.5*rou*V0^2*S));
if elevator>24
    elevator=24;
elseif elevator<-24
    elevator=-24;
end
Cm0=interp2(given_Elevator,given_Alpha,given_Cm,elevator,r2dalpha,'spline');
Cm=Cm0;
