clc
clear all
close all

A = [0 0 1 0.1; -1 0 0 0.5; 0 -1 0 0.3; 0 0 0 1]
B = [0 0 1 0.3;  0 1 0 0.3; -1 0 0 0.2; 0 0 0 1]
C = [0 1 0 -0.3; 0 0 -1 -0.25; -1 0 0 0.25; 0 0 0 1]

sampling_rate = 0.002;

nA=[A(1,1);A(2,1);A(3,1)];
oA=[A(1,2);A(2,2);A(3,2)];
aA=[A(1,3);A(2,3);A(3,3)];
pA=[A(1,4);A(2,4);A(3,4)];
nB=[B(1,1);B(2,1);B(3,1)];
oB=[B(1,2);B(2,2);B(3,2)];
aB=[B(1,3);B(2,3);B(3,3)];
pB=[B(1,4);B(2,4);B(3,4)];
nC=[C(1,1);C(2,1);C(3,1)];
oC=[C(1,2);C(2,2);C(3,2)];
aC=[C(1,3);C(2,3);C(3,3)];
pC=[C(1,4);C(2,4);C(3,4)];

r=1;
x = dot(nA, (pB - pA));
y = dot(oA, (pB - pA));
z = dot(aA, (pB - pA));
psi = atan2(dot(oA,aB), dot(nA, aB));
temp = sqrt(dot(nA, aB)^2 + dot(oA, aB)^2);
theta = atan2(temp, dot(aA, aB));
V_r_theta = 1-cos(r*theta);
sin_phi = -sin(psi)*cos(psi)*V_r_theta*dot(nA, nB) + (cos(psi)^2*V_r_theta+cos(theta))*dot(oA, nB) - sin(psi)*sin(theta)*dot(aA, nB);
cos_phi = -sin(psi)*cos(psi)*V_r_theta*dot(nA, oB) + (cos(psi)^2*V_r_theta+cos(theta))*dot(oA, oB) - sin(psi)*sin(theta)*dot(aA, oB);
phi = atan2(sin_phi, cos_phi);

%------------從A到A'--------------
dataA = 1;
for t=-0.5:sampling_rate:-0.2
    r=(t+0.5)/0.5;
    dx=x*r;
    dy=y*r;
    dz=z*r;
    dsi=psi;
    dtheta=theta*r;
    dphi=phi*r;   
    S_psi=sin(psi);
    C_psi=cos(psi);
    S_theta=sin(dtheta);
    C_theta=cos(dtheta);
    V_theta=1-C_theta;
    S_phi=sin(dphi);
    C_phi=cos(dphi);
    
    Tr = [1 0 0 dx; 0 1 0 dy; 0 0 1 dz; 0 0 0 1];
    Rar = [S_psi^2*V_theta+C_theta, -S_psi*C_psi*V_theta,C_psi*S_theta, 0;
          -S_psi*C_psi*V_theta,C_psi^2*V_theta+C_theta, S_psi*S_theta, 0;
          -C_psi*S_theta, -S_psi*S_theta, C_theta, 0;
           0, 0, 0, 1];
    Ror = [C_phi, -S_phi, 0, 0;
           S_phi, C_phi, 0, 0;
           0, 0, 1, 0;
           0, 0, 0, 1];
    Dr = Tr*Rar*Ror;
    
    pA_B(:,:,dataA)=A*Dr
    xA_B(:,dataA)=pA_B(1,4,dataA)
    yA_B(:,dataA)=pA_B(2,4,dataA)
    zA_B(:,dataA)=pA_B(3,4,dataA)
    axA(:,dataA)=pA_B(1,3,dataA)
    ayA(:,dataA)=pA_B(2,3,dataA)
    azA(:,dataA)=pA_B(3,3,dataA)
    oxA(:,dataA)=pA_B(1,2,dataA)
    oyA(:,dataA)=pA_B(2,2,dataA)
    ozA(:,dataA)=pA_B(3,2,dataA)
    nxA(:,dataA)=pA_B(1,1,dataA)
    nyA(:,dataA)=pA_B(2,1,dataA)
    nzA(:,dataA)=pA_B(3,1,dataA)
    
    dataA=dataA+1;
end

%----------從A'到B'------
A2=pA_B(:,:,dataA-1);
nA2=[A2(1,1);A2(2,1);A2(3,1)];
oA2=[A2(1,2);A2(2,2);A2(3,2)];
aA2=[A2(1,3);A2(2,3);A2(3,3)];
pA2=[A2(1,4);A2(2,4);A2(3,4)];

xA=nB'*(pA2-pB);
yA=oB'*(pA2-pB);
zA=aB'*(pA2-pB);
psiA=atan2(oB'*aA2,nB'*aA2);
thetaA=atan2(sqrt((nB'*aA2)^2+(oB'*aA2)^2),aB'*aA2);
SphiA=-sin(psiA)*cos(psiA)*(1-cos(thetaA))*(nB'*nA2)+((cos(psiA))^2*(1-cos(thetaA))+cos(thetaA))*(oB'*nA2)-sin(psiA)*sin(thetaA)*(aB'*nA2);
CphiA=-sin(psiA)*cos(psiA)*(1-cos(thetaA))*(nB'*oA2)+((cos(psiA))^2*(1-cos(thetaA))+cos(thetaA))*(oB'*oA2)-sin(psiA)*sin(thetaA)*(aB'*oA2);
phiA=atan2(SphiA,CphiA);

xC=nB'*(pC-pB);
yC=oB'*(pC-pB);
zC=aB'*(pC-pB);
psiC=atan2(oB'*aC,nB'*aC);
thetaC=atan2(sqrt((nB'*aC)^2+(oB'*aC)^2),aB'*aC);
SphiC=-sin(psiC)*cos(psiC)*(1-cos(thetaC))*(nB'*nC)+((cos(psiC))^2*(1-cos(thetaC))+cos(thetaC))*(oB'*nC)-sin(psiC)*sin(thetaC)*(aB'*nC);
CphiC=-sin(psiC)*cos(psiC)*(1-cos(thetaC))*(nB'*oC)+((cos(psiC))^2*(1-cos(thetaC))+cos(thetaC))*(oB'*oC)-sin(psiC)*sin(thetaC)*(aB'*oC);
phiC=atan2(SphiC,CphiC);

if abs(psiC-psiA)>pi/2
    psiA=psiA+pi;
    thetaA=-thetaA;
end


dataB=1;
for t=(-0.2+sampling_rate):sampling_rate:(0.2-sampling_rate)
    h=(t+0.2)/(0.2+0.2);
    dx_B=((xC*0.2/0.5+xA)*(2-h)*h^2-2*xA)*h+xA;
    dy_B=((yC*0.2/0.5+yA)*(2-h)*h^2-2*yA)*h+yA;
    dz_B=((zC*0.2/0.5+zA)*(2-h)*h^2-2*zA)*h+zA;
    dpsi_B=(psiC-psiA)*h+psiA;
    dtheta_B=((thetaC*0.2/0.5+thetaA)*(2-h)*h^2-2*thetaA)*h+thetaA;
    dphi_B=((phiC*0.2/0.5+phiA)*(2-h)*h^2-2*phiA)*h+phiA;
 
    S_psi=sin(dpsi_B);
    C_psi=cos(dpsi_B);
    S_theta=sin(dtheta_B);
    C_theta=cos(dtheta_B);
    V_theta=1-C_theta;
    S_phi=sin(dphi_B);
    C_phi=cos(dphi_B);
    
    Tr = [1 0 0 dx_B; 0 1 0 dy_B; 0 0 1 dz_B; 0 0 0 1];
    Rar = [S_psi^2*V_theta+C_theta, -S_psi*C_psi*V_theta,C_psi*S_theta, 0;
        -S_psi*C_psi*V_theta,C_psi^2*V_theta+C_theta, S_psi*S_theta, 0;
        -C_psi*S_theta, -S_psi*S_theta, C_theta, 0;
        0, 0, 0, 1];
    Ror = [C_phi, -S_phi, 0, 0;
        S_phi, C_phi, 0, 0;
        0, 0, 1, 0;
        0, 0, 0, 1];
    Dr_B = Tr*Rar*Ror;                    
  
    p_B(:,:,dataB)=B*Dr_B;
    x_B(:,dataB)=p_B(1,4,dataB);
    y_B(:,dataB)=p_B(2,4,dataB);
    z_B(:,dataB)=p_B(3,4,dataB); 
    axB(:,dataB)=p_B(1,3,dataB)
    ayB(:,dataB)=p_B(2,3,dataB)
    azB(:,dataB)=p_B(3,3,dataB)
    dataB=dataB+1;
end

%---------從B'到C---------------
dataC=1;
for t=0.2:sampling_rate:0.5
    h=t/0.5;
    dx_C=xC*h;
    dy_C=yC*h;
    dz_C=zC*h;
    dpsi_C=psiC;
    dtheta_C=thetaC*h;
    dphi_C=phiC*h;
 
    S_psi=sin(dpsi_C);
    C_psi=cos(dpsi_C);
    S_theta=sin(dtheta_C);
    C_theta=cos(dtheta_C);
    V_theta=1-C_theta;
    S_phi=sin(dphi_C);
    C_phi=cos(dphi_C);
        
    Tr = [1 0 0 dx_C; 0 1 0 dy_C; 0 0 1 dz_C; 0 0 0 1];
    Rar = [S_psi^2*V_theta+C_theta, -S_psi*C_psi*V_theta,C_psi*S_theta, 0;
        -S_psi*C_psi*V_theta,C_psi^2*V_theta+C_theta, S_psi*S_theta, 0;
        -C_psi*S_theta, -S_psi*S_theta, C_theta, 0;
        0, 0, 0, 1];
    Ror = [C_phi, -S_phi, 0, 0;
        S_phi, C_phi, 0, 0;
        0, 0, 1, 0;
        0, 0, 0, 1];
    Dr_C = Tr*Rar*Ror;
    
    p_C(:,:,dataC)=B*Dr_C;
    x_C(:,dataC)=p_C(1,4,dataC);
    y_C(:,dataC)=p_C(2,4,dataC);
    z_C(:,dataC)=p_C(3,4,dataC); 
    axC(:,dataC)=p_C(1,3,dataC)
    ayC(:,dataC)=p_C(2,3,dataC)
    azC(:,dataC)=p_C(3,3,dataC)
    oxC(:,dataC)=p_C(1,2,dataC)
    oyC(:,dataC)=p_C(2,2,dataC)
    ozC(:,dataC)=p_C(3,2,dataC)
    nxC(:,dataC)=p_C(1,1,dataC)
    nyC(:,dataC)=p_C(2,1,dataC)
    nzC(:,dataC)=p_C(3,1,dataC)
    dataC=dataC+1;
end

%-------位置--------
X=[xA_B*100 x_B*100 x_C*100];
Y=[yA_B*100 y_B*100 y_C*100];
Z=[zA_B*100 z_B*100 z_C*100];
t=-0.5:sampling_rate:0.5;
figure(1)
subplot(3,3,1);
plot(t,X);
axis([-0.5 0.5 -30 30])
title('position of x');
grid
subplot(3,3,4);
plot(t,Y);
axis([-0.5 0.5 -40 60])
ylabel('Position(cm)')
title('position of y');
grid
subplot(3,3,7);
plot(t,Z);
axis([-0.5 0.5 20 30])
title('position of z');
xlabel('Time(s)')
grid

%~~~~~~~~~速度~~~~~~~~~~~~~~~
dt=t(2:501);
dX=diff(X)/sampling_rate;
dY=diff(Y)/sampling_rate;
dZ=diff(Z)/sampling_rate;
subplot(3,3,2);
plot(dt,dX);
axis([-0.5 0.5 -150 50])
title('velocity of x');
grid
subplot(3,3,5);
plot(dt,dY);
axis([-0.5 0.5 -120 -20])
title('velocity of y');
grid
subplot(3,3,8);
plot(dt,dZ);
axis([-0.5 0.5 -20 20])
title('velocity of z');
xlabel('Time(s)')
ylabel('Velocity(cm/s)');
grid
%~~~~~~~~加速度~~~~~~~~~~~~~~
dt2=t(3:501);
dX2=diff(dX)/sampling_rate;
dY2=diff(dY)/sampling_rate;
dZ2=diff(dZ)/sampling_rate;
subplot(3,3,3);
plot(dt2,dX2);
axis([-0.5 0.5 -600 200])
title('acceleration of x');
grid
subplot(3,3,6);
plot(dt2,dY2);
axis([-0.5 0.5 -300 100])
title('acceleration of y');
grid
subplot(3,3,9);
plot(dt2,dZ2);
axis([-0.5 0.5 -50 150])
title('acceleration of z');
xlabel('Time(s)')
ylabel('Acceleration(cm/s^2)');
grid
%-------3D軌跡圖-------
figure(2)
plot3(xA_B*100,yA_B*100,zA_B*100,x_B*100,y_B*100,z_B*100,x_C*100,y_C*100,z_C*100,'LineWidth',1.5);

hold on
plot3([10 10],[50 40],[30 30],'r-');
hold on
plot3([10 10],[50 50],[30 20],'g-');
hold on
plot3([10 20],[50 50],[30 30],'b-');

hold on
plot3([30 30],[30 30],[20 10],'r-');
hold on
plot3([30 30],[30 40],[20 20],'g-');
hold on
plot3([30 40],[30 30],[20 20],'b-');

hold on
plot3([-30 -30],[-25 -25],[25 15],'r-');
hold on
plot3([-30 -20],[-25 -25],[25 25],'g-');
hold on
plot3([-30 -30],[-25 -35],[25 25],'b-');

hold on
plot3([10 30],[50 30],[30 20],'k:');
hold on
plot3([30 -30],[30 -25],[20 25],'k:');

xlabel('x(cm)');
ylabel('y(cm)');
zlabel('z(cm)');
text(10,50,30,'A(10,50,30)');
text(22,38,24,'A''(22,38,24)');
text(30,30,20,'B(30,30,20)');
text(6,8,22,'B''(6,8,22)');
text(-30,-25,25,'C(-30,-25,25)');
axis([-40 40 -40 60 10 30 ])
grid
title('3D path of Cartesion Motion')
%--------加方向的3D軌跡圖-----------
figure(3)
plot3(xA_B*100,yA_B*100,zA_B*100,x_B*100,y_B*100,z_B*100,x_C*100,y_C*100,z_C*100,'LineWidth',1.5);
xlabel('x(cm)');
ylabel('y(cm)');
zlabel('z(cm)');
text(10,50,30,'A(10,50,30)');
text(22,38,24,'A''(22,38,24)');
text(30,30,20,'B(30,30,20)');
text(6,8,22,'B''(6,8,22)');
text(-30,-25,25,'C(-30,-25,25)');
axis([-40 40 -40 60 10 30 ])
grid
title('3D path of Cartesion Motion')


s=1;

for t1=-0.5:sampling_rate:-0.2
hold on
x=[xA_B(s)*100,xA_B(s)*100+5*axA(s)]
y=[yA_B(s)*100,yA_B(s)*100+5*ayA(s)]
z=[zA_B(s)*100,zA_B(s)*100+5*azA(s)]
plot3(x,y,z,'-');
s=s+1;
end

s=1;
for t2=(-0.2+sampling_rate):sampling_rate:(0.2-sampling_rate)
hold on
x=[x_B(s)*100,x_B(s)*100+5*axB(s)]
y=[y_B(s)*100,y_B(s)*100+5*ayB(s)]
z=[z_B(s)*100,z_B(s)*100+5*azB(s)]
plot3(x,y,z,'-');
s=s+1;
end

s=1;
for t3=0.2:sampling_rate:0.5
hold on
x=[x_C(s)*100,x_C(s)*100+5*axC(s)]
y=[y_C(s)*100,y_C(s)*100+5*ayC(s)]
z=[z_C(s)*100,z_C(s)*100+5*azC(s)]
plot3(x,y,z,'-');
s=s+1;
end

hold on
plot3([10 10],[50 40],[30 30],'r-');
hold on
plot3([10 10],[50 50],[30 20],'g-');
hold on
plot3([10 20],[50 50],[30 30],'b-');

hold on
plot3([30 30],[30 30],[20 10],'r-');
hold on
plot3([30 30],[30 40],[20 20],'g-');
hold on
plot3([30 40],[30 30],[20 20],'b-');

hold on
plot3([-30 -30],[-25 -25],[25 15],'r-');
hold on
plot3([-30 -20],[-25 -25],[25 25],'g-');
hold on
plot3([-30 -30],[-25 -35],[25 25],'b-');

hold on
plot3([10 30],[50 30],[30 20],'k:');
hold on
plot3([30 -30],[30 -25],[20 25],'k:');

