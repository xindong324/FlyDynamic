function Clb = getClb(alpha0)
low = 0;
high = 1;
Clbl = getCl(alpha0,low);
Clbh = getCl(alpha0,high);
Clb = Clbh-Clbl;