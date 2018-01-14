function Cxde = getCxde(alpha,delte)
low = delte-0.01;
high = delte+0.01;
Cxdel = getCx(alpha,low);
Cxdeh = getCx(alpha,high);
Cxde = (Cxdeh-Cxdel)/0.02;