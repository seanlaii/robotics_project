function [p] = kinematics(joint_varaible);
dtor = pi/180;  %degree to rad
rtod = 180/pi;  %rad to degree

 

%if order == 1
    thetas=joint_varaible
    t1 = thetas(1);
    t2 = thetas(2);
    t3 = thetas(3);
    t4 = thetas(4);
    t5 = thetas(5);
    t6 = thetas(6);
    
      A1 = [cos(t1*dtor)  0  -sin(t1*dtor)  0.12*cos(t1*dtor) ;
            sin(t1*dtor)  0   cos(t1*dtor)  0.12*sin(t1*dtor) ;
            0            -1        0               0 ;
            0             0        0               1];
        
      A2 = [cos(t2*dtor)  -sin(t2*dtor)   0  0.25*cos(t2*dtor) ;
            sin(t2*dtor)   cos(t2*dtor)   0  0.25*sin(t2*dtor) ;
            0                    0        1         0 ;
            0                    0        0         1];
        
      A3 = [cos(t3*dtor)  -sin(t3*dtor)   0  0.26*cos(t3*dtor) ;
            sin(t3*dtor)   cos(t3*dtor)   0  0.26*sin(t3*dtor) ;
            0                    0        1         0 ;
            0                    0        0         1];
        
      A4 = [cos(t4*dtor)  0  -sin(t4*dtor)  0 ;
            sin(t4*dtor)  0   cos(t4*dtor)  0 ;
            0            -1        0        0 ;
            0             0        0        1];
        
      A5 = [cos(t5*dtor)  0   sin(t5*dtor)  0 ;
            sin(t5*dtor)  0  -cos(t5*dtor)  0 ;
            0             1        0        0 ;
            0             0        0        1];
        
      A6 = [cos(t6*dtor)  -sin(t6*dtor)   0  0 ;
            sin(t6*dtor)  cos(t6*dtor)    0  0 ;
            0                   0         1  0 ;
            0                   0         0  1]; 
        
    T6 = A1*A2*A3*A4*A5*A6;
    nx = T6(1,1);
    ny = T6(2,1);
    nz = T6(3,1);  
    ox = T6(1,2);
    oy = T6(2,2);
    oz = T6(3,2); 
    ax = T6(1,3);
    ay = T6(2,3);
    az = T6(3,3); 
    px = T6(1,4);
    py = T6(2,4);
    pz = T6(3,4); 
   
    phi = atan2(ay,ax)*rtod;
    theta = atan2(sqrt((ax)^2+(ay)^2),az)*rtod;
    psi = atan2(oz,-nz)*rtod;
    
    p = [px, py, pz, nx,ny,nz,ox,oy,oz,ax,ay,az]';
%     fprintf('Cartesian pont \n');
%     fprintf('<n.o.a.p>\n');
%     fprintf('%8.4f  %8.4f  %8.4f  %8.4f\n',nx,ox,ax,px);
%     fprintf('%8.4f  %8.4f  %8.4f  %8.4f\n',ny,oy,ay,py);
%     fprintf('%8.4f  %8.4f  %8.4f  %8.4f\n',nz,oz,az,pz);
%     fprintf('%8.4f  %8.4f  %8.4f  %8.4f\n',0,0,0,1);
%     fprintf('(x,y,z, phi,theta,phi)=(%f, %f, %f, %f, %f, %f) \n',px,py,pz,phi,theta,psi);  
    if abs(t1)>150
    %    fprintf('Theta1 is out of range \n');
    end
    if t2>110 || t2<-30
    %    fprintf('Theta2 is out of range \n');
    end
    if t3>0 || t3<-120
    %    fprintf('Theta3 is out of range \n');
    end
    if abs(t4)>110
    %    fprintf('Theta4 is out of range \n');
    end
    if abs(t5)>180
  %      fprintf('Theta5 is out of range \n');
    end
    if abs(t6)>180
       % fprintf('Theta6 is out of range \n');
    end

end