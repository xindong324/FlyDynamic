function Cxa = getCxa(alpha,delte)

low = alpha - 0.1;
high = alpha + 0.1;
Cxl = getCx(low,delte);
Cxh = getCx(high,delte);
Cxa = (Cxh-Cxl)/0.2;