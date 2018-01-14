function Throtte=trimfunction1(Throtte)%油门配平函数
global T given_height given_throtte_000 given_throtte_077 given_throtte_100 given_mah H Mah0
Interp_throtte_000=interp2(given_height,given_mah,given_throtte_000,H/0.3048,Mah0,'spline')*(14.5939*0.3048);   %(slug x ft)/s^2 ->(kg x m)/s^2
Interp_throtte_077=interp2(given_height,given_mah,given_throtte_077,H/0.3048,Mah0,'spline')*(14.5939*0.3048);  % 英制推力单位向公制转换
Interp_throtte_100=interp2(given_height,given_mah,given_throtte_100,H/0.3048,Mah0,'spline')*(14.5939*0.3048);
matrix_interp=[Interp_throtte_000	Interp_throtte_077	Interp_throtte_100];
throtte_value=[0 0.77 1];
Throtte=interp1(throtte_value,matrix_interp,Throtte)-T;
