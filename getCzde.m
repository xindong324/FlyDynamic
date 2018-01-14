function Czde = getCzde(alpha,delte)
low = delte-0.1;
high = delte+0.1;
Czdel = getCz(alpha,0,low);
Czdeh = getCz(alpha,0,high);
Czde = (Czdeh-Czdel)/0.2;