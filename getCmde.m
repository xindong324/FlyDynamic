function Cmde = getCmde(alpha,delte)
low = delte-0.1;
high = delte+0.1;
Cmdel = getCm(alpha,low);
Cmdeh = getCm(alpha,high);
Cmde = (Cmdeh-Cmdel)/0.2;
