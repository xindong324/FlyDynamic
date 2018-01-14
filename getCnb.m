function Cnb = getCnb(alpha0)
low = 0;
high = 1;
Cnbl = getCn(alpha0,low);
Cnbh = getCn(alpha0,high);
Cnb = Cnbh-Cnbl;