function Cma = getCma(alpha,delte)

low = alpha - 0.1;
high = alpha + 0.1;
Cml = getCm(low,delte);
Cmh = getCm(high,delte);
Cma = (Cmh-Cml)/0.2;