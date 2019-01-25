clc
clear all
close all
%經inverse kinematics後選擇工作範圍內的一組解----------
theta_A=[78.6901,  -21.9802,   -30.5766,   52.5567,   168.6901,   0.0000];
theta_B=[45.0000,   12.2384,   -88.9084,   76.6700,   135.0000,   90.0000];
theta_C=[-140.1944,   2.1035,   -87.5413,   85.4378,   39.8056,   90.0000];

sampling_rate = 0.002;

thetaA=theta_A(1,:)';                          
thetaB=theta_B(1,:)';    
thetaC=theta_C(1,:)';    

s=1;
%從A到A'
for t=-0.5:sampling_rate:-0.2
    qA(:,s)=thetaA+(thetaB-thetaA)/0.5*(t+0.5);                 
    vA(:,s)=(thetaB-thetaA)/0.5;
    accA(:,s)=[0;0;0;0;0;0];
    s=s+1;
end

%從A'到B'
thetaA2=thetaA+(thetaB-thetaA)/0.5*(0.5-0.2);                        
dB=thetaA2-thetaB;
dC=thetaC-thetaB;
s=1;
for t=(-0.2+sampling_rate):sampling_rate:(0.2-sampling_rate)
    h=(t+0.2)/(2*0.2);
    qB(:,s)=((dC*0.2/0.5+dB)*(2-h)*h^2-2*dB)*h+dB+thetaB;  
    
    vB(:,s)=((dC*0.2/0.5+dB)*(1.5-h)*2*h^2-dB)/0.2;
    accB(:,s)=(dC*0.2/0.5+dB)*(1-h)*3*h/0.2^2;
    s=s+1;
end

%從B'到C
s=1;
for t=0.2:sampling_rate:0.5;
    qC(:,s)=dC*t/0.5+thetaB;
    vC(:,s)=dC/0.5;
    accC(:,s)=[0;0;0;0;0;0];
    s=s+1;
end

%------角度--------------------------
figure(1)
t=-0.5:sampling_rate:0.5;
theta1=[qA(1,:) qB(1,:) qC(1,:)];                     
subplot(3,2,1);
plot(t,theta1);
axis([-0.5 0.5 -200 200])
grid
title('joint1');
ylabel('Angle(。)');
theta2=[qA(2,:) qB(2,:) qC(2,:)];                   
subplot(3,2,2);
plot(t,theta2);
axis([-0.5 0.5 -50 50])
grid
title('joint2');
ylabel('Angle(。)');
theta3=[qA(3,:) qB(3,:) qC(3,:)];                     
subplot(3,2,3);
plot(t,theta3);
axis([-0.5 0.5 -100 0])
grid
title('joint3');
ylabel('Angle(。)');
theta4=[qA(4,:) qB(4,:) qC(4,:)];                    
subplot(3,2,4);
plot(t,theta4);
axis([-0.5 0.5 50 100])
grid
title('joint4');
ylabel('Angle(。)');
theta5=[qA(5,:) qB(5,:) qC(5,:)];                    
subplot(3,2,5);
plot(t,theta5);
axis([-0.5 0.5 0 200])
grid
title('joint5');
ylabel('Angle(。)');
theta6=[qA(6,:) qB(6,:) qC(6,:)];                     
subplot(3,2,6);
plot(t,theta6);
axis([-0.5 0.5 0 100])
grid
title('joint6');
ylabel('Angle(。)');

%----------速度-------------------------
figure(2)
subplot(3,2,1)
plot(t,[vA(1,:) vB(1,:) vC(1,:)]);  
axis([-0.5 0.5 -400 0])
grid
title('joint1');
ylabel('Angular Velocity');
subplot(3,2,2)
plot(t,[vA(2,:) vB(2,:) vC(2,:)]); 
axis([-0.5 0.5 -100 100])
grid
title('joint2');
ylabel('Angular Velocity');
subplot(3,2,3)
plot(t,[vA(3,:) vB(3,:) vC(3,:)]); 
axis([-0.5 0.5 -200 200])
grid
title('joint3');
ylabel('Angular Velocity');
subplot(3,2,4)
plot(t,[vA(4,:) vB(4,:) vC(4,:)]);
axis([-0.5 0.5 0 50])
grid
title('joint4');
ylabel('Angular Velocity');
subplot(3,2,5)
plot(t,[vA(5,:) vB(5,:) vC(5,:)]);  
axis([-0.5 0.5 -200 0])
grid
title('joint5');
ylabel('Angular Velocity');
subplot(3,2,6)
plot(t,[vA(6,:) vB(6,:) vC(6,:)]);   
axis([-0.5 0.5 0 200])
grid
title('joint6');
ylabel('Angular Velocity');

%-----------加速度-----------------------
figure(3)
subplot(3,2,1)
plot(t,[accA(1,:) accB(1,:) accC(1,:)]);  
axis([-0.5 0.5 -2000 0])
grid
title('joint1');
ylabel('Angular Acceleration');
subplot(3,2,2)
plot(t,[accA(2,:) accB(2,:) accC(2,:)]); 
axis([-0.5 0.5 -400 0])
grid
title('joint2');
ylabel('Angular Acceleration');
subplot(3,2,3)
plot(t,[accA(3,:) accB(3,:) accC(3,:)]); 
axis([-0.5 0.5 0 500])
grid
title('joint3');
ylabel('Angular Acceleration');
subplot(3,2,4)
plot(t,[accA(4,:) accB(4,:) accC(4,:)]);
axis([-0.5 0.5 -200 0])
grid
title('joint4');
ylabel('Angular Acceleration');
subplot(3,2,5)
plot(t,[accA(5,:) accB(5,:) accC(5,:)]);  
axis([-0.5 0.5 -500 0])
grid
title('joint5');
ylabel('Angular Acceleration');
subplot(3,2,6)
plot(t,[accA(6,:) accB(6,:) accC(6,:)]);   
axis([-0.5 0.5 -1000 0])
grid
title('joint6');
ylabel('Angular Acceleration');

%A到A'
s=1;
for t1=-0.5:sampling_rate:-0.2
    %用forward kinematics
    p1=kinematics(qA(:,s)');                   
    x1(s)=p1(1,1)*100;
    y1(s)=p1(2,1)*100;
    z1(s)=p1(3,1)*100;
    nx1(s)=p1(4,1);
    ny1(s)=p1(5,1);
    nz1(s)=p1(6,1);
    ox1(s)=p1(7,1);
    oy1(s)=p1(8,1);
    oz1(s)=p1(9,1);
    ax1(s)=p1(10,1);
    ay1(s)=p1(11,1);
    az1(s)=p1(12,1);
    s=s+1;
end
%A'到B'
s=1;
for t2=(-0.2+sampling_rate):sampling_rate:(0.2-sampling_rate)
    %用forward kinematics
    p2=kinematics(qB(:,s)');
    x2(s)=p2(1,1)*100;
    y2(s)=p2(2,1)*100;
    z2(s)=p2(3,1)*100;
    nx2(s)=p2(4,1);
    ny2(s)=p2(5,1);
    nz2(s)=p2(6,1);
    ox2(s)=p2(7,1);
    oy2(s)=p2(8,1);
    oz2(s)=p2(9,1);
    ax2(s)=p2(10,1);
    ay2(s)=p2(11,1);
    az2(s)=p2(12,1);
    s=s+1;
end
%B'到C'
s=1;
for t3=0.2:sampling_rate:0.5
    %用forward kinematics
    p3=kinematics(qC(:,s)');
    x3(s)=p3(1,1)*100;
    y3(s)=p3(2,1)*100;
    z3(s)=p3(3,1)*100;
    nx3(s)=p3(4,1);
    ny3(s)=p3(5,1);
    nz3(s)=p3(6,1);
    ox3(s)=p3(7,1);
    oy3(s)=p3(8,1);
    oz3(s)=p3(9,1);
    ax3(s)=p3(10,1);
    ay3(s)=p3(11,1);
    az3(s)=p3(12,1);
    s=s+1;
end


%-----3D軌跡圖----
figure(4)
plot3(x1,y1,z1,x2,y2,z2,x3,y3,z3,'LineWidth',1.5);
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
text(24.6477,40.1837,24.5697,'A''(24.6477,40.1837,24.5697)');
text(30,30,20,'B(30,30,20)');
text(36.2579,-19.5531,22.0396,'B''(35.9906,-20.0138,22.0598)');
text(-30,-25,25,'C(-30,-25,25)');
axis([-50 50 -50 50 10 35 ])

grid
title('3D path of Joint Motion')

%--3D軌跡圖加方向----
figure(5)
plot3(x1,y1,z1,x2,y2,z2,x3,y3,z3,'LineWidth',1.5);

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
text(24.6477,40.1837,24.5697,'A''(24.6477,40.1837,24.5697)');
text(30,30,20,'B(30,30,20)');
text(36.2579,-19.5531,22.0396,'B''(35.9906,-20.0138,22.0598)');
text(-30,-25,25,'C(-30,-25,25)');
axis([-50 50 -50 50 10 35 ])

grid
title('3D path of Joint Motion')

s=1;
for t1=-0.5:sampling_rate:-0.2
hold on
x=[x1(s),x1(s)+5*ax1(s)]
y=[y1(s),y1(s)+5*ay1(s)]
z=[z1(s),z1(s)+5*az1(s)]
plot3(x,y,z,'-');
s=s+1;
end

s=1;
for t2=(-0.2+sampling_rate):sampling_rate:(0.2-sampling_rate)
hold on
x=[x2(s),x2(s)+5*ax2(s)]
y=[y2(s),y2(s)+5*ay2(s)]
z=[z2(s),z2(s)+5*az2(s)]
plot3(x,y,z,'-');
s=s+1;
end

s=1;
for t3=0.2:sampling_rate:0.5
hold on
x=[x3(s),x3(s)+5*ax3(s)]
y=[y3(s),y3(s)+5*ay3(s)]
z=[z3(s),z3(s)+5*az3(s)]
plot3(x,y,z,'-');
s=s+1;
end


