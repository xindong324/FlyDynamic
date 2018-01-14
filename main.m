clear
clc
kg2slug = 0.068522*3.2808399;
h = 10000;
v = 190;

S = 27.87;
cbar = 3.45;
band = 9.144;
m = 9298.643585;
Ix = 12874.8446;
Iy = 75673.6077;
Iz = 85552.095;
Ixz = 1331.4130;
dg2r = 180/pi;


[rou g va] = getRou(h);
mach = v/va;


%root = 8.4766
[root delte]= FindRoot(0,11,1e-5);
root = 5.36251484934303;
delte = -0.496968582660390;
deltt = 0.3236;
alpha_pi = root*pi/180;
u = v*cos(alpha_pi);
dpress=0.5*rou*u*u*S;
W = m*g;
Cw = W/dpress;
%myr = getCm(root,delte)
Cx = getCx(root,delte);
T0 = W*sin(alpha_pi)-Cx*dpress;
%T0slug = T0*kg2slug;%»»ËãÎªslug
deltt = FindRT(0,1,T0,1e-5);
 Xdt = mytdt(mach,h,deltt);
 Xu=-1*rou*u*S*Cw*sin(alpha_pi);
 Cx_a= getCxa(root,delte)*dg2r;

 Xw=dpress*Cx_a/u;
 Cxq = getCxq(root);
 Xq=0.25*rou*cbar*S*Cxq*u;
 Cxde = getCxde(root,delte);
 Xde = dpress*Cxde;
 
 Cz = getCz(root,0,delte);
 
 Zu = -1*rou*u*S*Cw*cos(alpha_pi);
 Cza = getCza(root,delte)*dg2r;
 Zw = 0.5*rou*u*S*Cza;
  Czq = getCzq(root);
 Zq=0.25*rou*u*cbar*S*Czq;
 Zdt = 0;
 Czde = getCzde(root,delte);
 Zde = dpress*Czde;
 
 %% varible M
Cm = getCm(root,delte);
 Mu = rou*u*S*cbar*Cm;
 Cma = getCma(root,delte)*dg2r;
 Mw = dpress*cbar*Cma/u ;
 Cmq = getCmq(root);
 Mq=0.25*rou*u*cbar*cbar*S*Cmq;
 Mdt = 0;
 Cmde = getCmde(root,delte);
 Mde = dpress*cbar*Cmde;
 
 Az = [Xu/m Xw/m Xq/m -1*g*cos(alpha_pi);
     Zu/m Zw/m Zq/m+u -1*g*sin(alpha_pi);
     Mu/Iy Mw/Iy Mq/Iy 0;
     0 0 1 0;];
 Bz = [Xdt/m Xde/m;Zdt/m Zde/m;Mdt/Iy Mde/Iy; 0 0];
 
 [Vz eigz] = eig(Az);
 %% heng xiang
 Cyb = -0.02;
 Cyda = 0.021/20;
 Cydr = 0.086/30;
[Cyp Cyr] = getCypr(root);
Yv = dpress*Cyb*dg2r/v;
Yp = 0.25*rou*v*S*band*Cyp;
Yr = 0.25*rou*v*S*band*Cyr;
Yda = dpress*Cyda;
Ydr = dpress*Cydr;

%% Cl
Clb = getClb(root);
[Clp Clr] = getClpr(root);
Clda = getClda(root,0);
Cldr = getCldr(root,0);

Lv = dpress*band*Clb*dg2r/v;
Lp = 0.25*rou*v*S*band*band*Clp;
Lr = 0.25*rou*v*S*band*band*Clr;
Lda = dpress*band*Clda;
Ldr = dpress*band*Cldr;

%% Cn
Cnb = getCnb(root);
[Cnp Cnr] = getCnpr(root);
Cnda = getCnda(root,0);
Cndr = getCndr(root, 0);


Nv = dpress*band*Cnb*dg2r/v;
Np =  0.25*rou*v*S*band*Cnp;
Nr =  0.25*rou*v*S*band*band*Cnr;
Nda = dpress*band*Cnda;
Ndr = dpress*band*Cndr;



%% hengxiang AB
ndown = Ix*Iz-Ixz*Ixz;
Ah = [Yv/m Yp/m Yr/m-v g*cos(alpha_pi);
    (Iz*Lv+Ixz*Nv)/ndown (Iz*Lp+Np*Ixz)/ndown (Iz*Lr+Nr*Ixz)/ndown 0;
    (Ixz*Lv+Ix*Nv)/ndown (Ixz*Lp+Np*Ix)/ndown (Ixz*Lr+Nr*Ix)/ndown 0;
    0 1 tan(alpha_pi) 0];
Bh = [Yda/m Ydr/m;
    (Iz*Lda+Nda*Ixz)/ndown (Iz*Ldr+Ndr*Ixz)/ndown;
    (Ixz*Lda+Nda*Ix)/ndown (Ixz*Ldr+Ndr*Ix)/ndown;
    0 0 ];
[Vh eigh] = eig(Ah);
