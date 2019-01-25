from math import cos ,sin ,atan2 ,sqrt, pi, acos
import numpy as np

min_angle = [-150, -30, -120, -110, -180, -180]
max_angle = [150, 100, 0, 110, 180, 180]
rad_perdeg = pi/180
solution = 0
def check_theta_range(mina, maxa, i, angle):  #check angle range
    if mina <= angle <= maxa:
        return 1
    print ("theta" + str(i) + " is out of the angle range, it need to be within the range of " + str(mina) + " to " + str(maxa))
    return 0
def arccos_d(value):  #Because the value might be bigger than 1, we need to check out the value in order to prevent the wrong answers. 
    if abs(value) > 1:
        return 10
    else:
        return acos(value)
        
service = 0
while(service != 3):
    service = int(input("Inpute the number of the desired service:\n1.Calculate forward kinematic \n2.Calculate inverse kinematic \n3.Exit\n"))
    in_the_range = 1
    if service == 1:
        try:
            t_list = []
            print("Input theta:")
            for i in range(6):
                a = float(input("theta" + str(i+1) + ": "))
                t_list.append(a)
            print("The angles you input:")
            for i in range(6):
                print("theta" + str(i+1) + ": " + str(t_list[i]) + " ")
            
            
            #Compute forward kinematics
            A1 = np.array([ 
            [cos(t_list[0] * rad_perdeg), 0, -1*sin(t_list[0] * rad_perdeg), 0.12*cos(t_list[0] * rad_perdeg)],
            [sin(t_list[0] * rad_perdeg), 0, cos(t_list[0] * rad_perdeg), 0.12*sin(t_list[0] * rad_perdeg)],
            [0, -1, 0, 0],
            [0, 0, 0, 1]]
            )

            A2 = np.array([ 
            [cos(t_list[1] * rad_perdeg), -1*sin(t_list[1] * rad_perdeg), 0, 0.25*cos(t_list[1] * rad_perdeg)],
            [sin(t_list[1] * rad_perdeg), cos(t_list[1] * rad_perdeg), 0, 0.25*sin(t_list[1] * rad_perdeg)],
            [0, 0, 1, 0],
            [0, 0, 0, 1]]
            )

            A3 = np.array([ 
            [cos(t_list[2] * rad_perdeg), -1*sin(t_list[2] * rad_perdeg), 0, 0.26*cos(t_list[2] * rad_perdeg)],
            [sin(t_list[2] * rad_perdeg), cos(t_list[2] * rad_perdeg), 0, 0.26*sin(t_list[2] * rad_perdeg)],
            [0, 0, 1, 0],
            [0, 0, 0, 1]]
            )

            A4 = np.array([ 
            [cos(t_list[3] * rad_perdeg), 0, -1*sin(t_list[3] * rad_perdeg), 0],
            [sin(t_list[3] * rad_perdeg), 0, cos(t_list[3] * rad_perdeg), 0],
            [0, -1, 0, 0],
            [0, 0, 0, 1]]
            )

            A5 = np.array([ 
            [cos(t_list[4] * rad_perdeg), 0, sin(t_list[4] * rad_perdeg), 0],
            [sin(t_list[4] * rad_perdeg), 0, -1*cos(t_list[4] * rad_perdeg), 0],
            [0, 1, 0, 0],
            [0, 0, 0, 1]]
            )

            A6 = np.array([ 
            [cos(t_list[5] * rad_perdeg), -1*sin(t_list[5] * rad_perdeg), 0, 0],
            [sin(t_list[5] * rad_perdeg), cos(t_list[5] * rad_perdeg), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]]
            )

            T = np.dot(np.dot(np.dot(np.dot(np.dot(A1,A2),A3),A4),A5),A6)
            n = T[0:3,0]
            o = T[0:3,1]
            a = T[0:3,2]
            p = T[0:3,3]

            #compute Euler angles
            phi = atan2(a[1], a[0])  
            theta = atan2((cos(phi)*a[0] + sin(phi)*a[1]), a[2])   
            psi = atan2(-1*sin(phi)*n[0]+cos(phi)*n[1], -1*sin(phi)*o[0]+cos(phi)*o[1])  
            
            print("(n, o, a, p) = ")
            print T
            print("\n(x, y, z, phi, theta, psi) = " + "(" + str(p[0]) + ", " + str(p[1]) + ", " + str(p[2]) + ", " + str(phi) + ", " + str(theta) + ", " + str(psi) + ")")
            for i in range(6):
                r = check_theta_range(min_angle[i], max_angle[i], i+1, t_list[i])
        except:
            print("Please enter the correct joint variables.")
    elif service == 2:
        
        n = []
        o = []
        a = []
        p = []
        joint_variables = []  
        print("Input n, o, a, p: ")
        for i in range(3):
            n.append(float(input("n" + chr(i+120) + ": ")))
            o.append(float(input("o" + chr(i+120) + ": ")))
            a.append(float(input("a" + chr(i+120) + ": "))) 
            p.append(float(input("p" + chr(i+120) + ": ")))
        #Compute inverse kinematics
        #Solution 1
        joint_variables.append(atan2(p[1], p[0]))
        v = p[0]*cos(joint_variables[0]) + p[1]*sin(joint_variables[0])-0.12
        ct3 = (v**2 + p[2]**2 - 0.26**2 - 0.25**2)/(0.5*0.26)
         
        joint_variables.insert(1, arccos_d(ct3))
        joint_variables.insert(1, ((atan2(-0.26*sin(joint_variables[1]), 0.25+0.26*cos(joint_variables[1])) - atan2(p[2], v))  ))
        joint_variables.insert(3, (atan2(-1*a[2], a[0]*cos(joint_variables[0])+a[1]*sin(joint_variables[0])) - joint_variables[1] - joint_variables[2])  )
        st5 = (a[0]*cos(joint_variables[0])+a[1]*sin(joint_variables[0])-a[2])/(sin(joint_variables[1]+joint_variables[2]+joint_variables[3])+cos(joint_variables[1]+joint_variables[2]+joint_variables[3]))
        ct5 = -1*a[0]*sin(joint_variables[0]) + a[1]*cos(joint_variables[0])
        joint_variables.insert(4, atan2(st5, ct5))
        st6 = (-1*o[0]*sin(joint_variables[0])+o[1]*cos(joint_variables[0]))/sin(joint_variables[4])
        ct6 = ((cos(joint_variables[4])*sin(joint_variables[1]+joint_variables[2]+joint_variables[3])*st6)-o[2])/cos(joint_variables[1]+joint_variables[2]+joint_variables[3])
        joint_variables.insert(5, atan2(st6, ct6))
        while((joint_variables[3]/rad_perdeg) >= 180 or (joint_variables[3]/rad_perdeg) <=-180): 
            if (joint_variables[3]/rad_perdeg) > 0:
                joint_variables[3]= (joint_variables[3]/rad_perdeg - 360)*rad_perdeg
            elif (joint_variables[3]/rad_perdeg) < 0:
                joint_variables[3]= (joint_variables[3]/rad_perdeg + 360)*rad_perdeg
        if arccos_d(ct3) != 10:
            solution += 1
            print ("Solution " + str(solution) + ": ")
            for k in range(6):
                result = check_theta_range(min_angle[k], max_angle[k], k+1, joint_variables[k]/rad_perdeg)
                print("Theta" + str(k+1) + "= " + str(joint_variables[k]/rad_perdeg))
                
            
        
        #Solution 2 (theta 1 + 180)
        joint_variables[0] = (atan2((-1)*p[1], (-1)*p[0]))
        v = p[0]*cos(joint_variables[0]) + p[1]*sin(joint_variables[0])-0.12
        ct3 = (v**2 + p[2]**2 - 0.26**2 - 0.25**2)/(0.5*0.26)
         
        joint_variables[2] = arccos_d(ct3)  
        joint_variables[1] = (atan2(-0.26*sin(joint_variables[2]), 0.25+0.26*cos(joint_variables[2])) - atan2(p[2], v))  
        joint_variables[3] = (atan2(-1*a[2], a[0]*cos(joint_variables[0])+a[1]*sin(joint_variables[0])) - joint_variables[1] - joint_variables[2])  
        st5 = (a[0]*cos(joint_variables[0])+a[1]*sin(joint_variables[0])-a[2])/(sin(joint_variables[1]+joint_variables[2]+joint_variables[3])+cos(joint_variables[1]+joint_variables[2]+joint_variables[3]))
        ct5 = -1*a[0]*sin(joint_variables[0]) + a[1]*cos(joint_variables[0])
        joint_variables[4] = atan2(st5, ct5)  
        st6 = (-1*o[0]*sin(joint_variables[0])+o[1]*cos(joint_variables[0]))/sin(joint_variables[4])
        ct6 = ((cos(joint_variables[4])*sin(joint_variables[1]+joint_variables[2]+joint_variables[3])*st6)-o[2])/cos(joint_variables[1]+joint_variables[2]+joint_variables[3])
        joint_variables[5] = atan2(st6, ct6) 
        while((joint_variables[3]/rad_perdeg) >= 180 or (joint_variables[3]/rad_perdeg) <=-180): 
            if (joint_variables[3]/rad_perdeg) > 0:
                joint_variables[3]= (joint_variables[3]/rad_perdeg - 360)*rad_perdeg
            elif (joint_variables[3]/rad_perdeg) < 0:
                joint_variables[3]= (joint_variables[3]/rad_perdeg + 360)*rad_perdeg
        if arccos_d(ct3) != 10:
            solution += 1
            print ("Solution " + str(solution) + ": ")
            for k in range(6):
                result = check_theta_range(min_angle[k], max_angle[k], k+1, joint_variables[k]/rad_perdeg)
                print("Theta" + str(k+1) + "= " + str(joint_variables[k]/rad_perdeg))
                
        #Solution 3 (theta 3 * -1 )
        joint_variables[0] = atan2(p[1], p[0])  
        v = p[0]*cos(joint_variables[0]) + p[1]*sin(joint_variables[0])-0.12
        ct3 = (v**2 + p[2]**2 - 0.26**2 - 0.25**2)/(0.5*0.26)
         
        joint_variables[2] = (arccos_d(ct3)  ) * (-1)
        joint_variables[1] = (atan2(-0.26*sin(joint_variables[2]), 0.25+0.26*cos(joint_variables[2])) - atan2(p[2], v))  
        joint_variables[3] = (atan2(-1*a[2], a[0]*cos(joint_variables[0])+a[1]*sin(joint_variables[0])) - joint_variables[1] - joint_variables[2])  
        st5 = (a[0]*cos(joint_variables[0])+a[1]*sin(joint_variables[0])-a[2])/(sin(joint_variables[1]+joint_variables[2]+joint_variables[3])+cos(joint_variables[1]+joint_variables[2]+joint_variables[3]))
        ct5 = -1*a[0]*sin(joint_variables[0]) + a[1]*cos(joint_variables[0])
        joint_variables[4] = atan2(st5, ct5)  
        st6 = (-1*o[0]*sin(joint_variables[0])+o[1]*cos(joint_variables[0]))/sin(joint_variables[4])
        ct6 = ((cos(joint_variables[4])*sin(joint_variables[1]+joint_variables[2]+joint_variables[3])*st6)-o[2])/cos(joint_variables[1]+joint_variables[2]+joint_variables[3])
        joint_variables[5] = atan2(st6, ct6)  
        while((joint_variables[3]/rad_perdeg) >= 180 or (joint_variables[3]/rad_perdeg) <=-180): 
            if (joint_variables[3]/rad_perdeg) > 0:
                joint_variables[3]= (joint_variables[3]/rad_perdeg - 360)*rad_perdeg
            elif (joint_variables[3]/rad_perdeg) < 0:
                joint_variables[3]= (joint_variables[3]/rad_perdeg + 360)*rad_perdeg
        if arccos_d(ct3) != 10:
            solution += 1
            print ("Solution " + str(solution) + ": ")
            for k in range(6):
                result = check_theta_range(min_angle[k], max_angle[k], k+1, joint_variables[k]/rad_perdeg)
                print("Theta" + str(k+1) + "= " + str(joint_variables[k]/rad_perdeg))
                
        #Solution 4 (theta 4 + 180)
        joint_variables[0] = (atan2(p[1], p[0])  )
        v = p[0]*cos(joint_variables[0]) + p[1]*sin(joint_variables[0])-0.12
        ct3 = (v**2 + p[2]**2 - 0.26**2 - 0.25**2)/(0.5*0.26)
         
        joint_variables[2] = arccos_d(ct3)  
        joint_variables[1] = (atan2(-0.26*sin(joint_variables[2]), 0.25+0.26*cos(joint_variables[2])) - atan2(p[2], v))  
        joint_variables[3] = ((atan2(a[2], -1*(a[0]*cos(joint_variables[0])+a[1]*sin(joint_variables[0]))) - joint_variables[1] - joint_variables[2]))
        st5 = (a[0]*cos(joint_variables[0])+a[1]*sin(joint_variables[0])-a[2])/(sin(joint_variables[1]+joint_variables[2]+joint_variables[3])+cos(joint_variables[1]+joint_variables[2]+joint_variables[3]))
        ct5 = -1*a[0]*sin(joint_variables[0]) + a[1]*cos(joint_variables[0])
        joint_variables[4] = atan2(st5, ct5)  
        st6 = (-1*o[0]*sin(joint_variables[0])+o[1]*cos(joint_variables[0]))/sin(joint_variables[4])
        ct6 = ((cos(joint_variables[4])*sin(joint_variables[1]+joint_variables[2]+joint_variables[3])*st6)-o[2])/cos(joint_variables[1]+joint_variables[2]+joint_variables[3])
        joint_variables[5] = atan2(st6, ct6) 
        while((joint_variables[3]/rad_perdeg) >= 180 or (joint_variables[3]/rad_perdeg) <=-180):
            if (joint_variables[3]/rad_perdeg) > 0:
                joint_variables[3]= (joint_variables[3]/rad_perdeg - 360)*rad_perdeg
            elif (joint_variables[3]/rad_perdeg) < 0:
                joint_variables[3]= (joint_variables[3]/rad_perdeg + 360)*rad_perdeg
        if arccos_d(ct3) != 10:
            solution += 1
            print ("Solution " + str(solution) + ": ")
            for k in range(6):
                result = check_theta_range(min_angle[k], max_angle[k], k+1, joint_variables[k]/rad_perdeg)
                print("Theta" + str(k+1) + "= " + str(joint_variables[k]/rad_perdeg))
                

        #Solution 5 (theta 1 + 180, theta 3 * -1)
        joint_variables[0] = (atan2((-1)*p[1], (-1)*p[0]))
        v = p[0]*cos(joint_variables[0]) + p[1]*sin(joint_variables[0])-0.12
        ct3 = (v**2 + p[2]**2 - 0.26**2 - 0.25**2)/(0.5*0.26)
         
        joint_variables[2] = (arccos_d(ct3)  ) * (-1)
        joint_variables[1] = (atan2(-0.26*sin(joint_variables[2]), 0.25+0.26*cos(joint_variables[2])) - atan2(p[2], v))  
        joint_variables[3] = (atan2(-1*a[2], a[0]*cos(joint_variables[0])+a[1]*sin(joint_variables[0])) - joint_variables[1] - joint_variables[2])  
        st5 = (a[0]*cos(joint_variables[0])+a[1]*sin(joint_variables[0])-a[2])/(sin(joint_variables[1]+joint_variables[2]+joint_variables[3])+cos(joint_variables[1]+joint_variables[2]+joint_variables[3]))
        ct5 = -1*a[0]*sin(joint_variables[0]) + a[1]*cos(joint_variables[0])
        joint_variables[4] = atan2(st5, ct5)  
        st6 = (-1*o[0]*sin(joint_variables[0])+o[1]*cos(joint_variables[0]))/sin(joint_variables[4])
        ct6 = ((cos(joint_variables[4])*sin(joint_variables[1]+joint_variables[2]+joint_variables[3])*st6)-o[2])/cos(joint_variables[1]+joint_variables[2]+joint_variables[3])
        joint_variables[5] = atan2(st6, ct6)  
        while((joint_variables[3]/rad_perdeg) >= 180 or (joint_variables[3]/rad_perdeg) <=-180):
            if (joint_variables[3]/rad_perdeg) > 0:
                joint_variables[3]= (joint_variables[3]/rad_perdeg - 360)*rad_perdeg
            elif (joint_variables[3]/rad_perdeg) < 0:
                joint_variables[3]= (joint_variables[3]/rad_perdeg + 360)*rad_perdeg
        if arccos_d(ct3) != 10:
            solution += 1
            print ("Solution " + str(solution) + ": ")
            for k in range(6):
                result = check_theta_range(min_angle[k], max_angle[k], k+1, joint_variables[k]/rad_perdeg)
                print("Theta" + str(k+1) + "= " + str(joint_variables[k]/rad_perdeg))
                
                        
        #Solution 6 (theta 1 + 180, theta 4 + 180)
        joint_variables[0] = (atan2((-1)*p[1], (-1)*p[0]))
        v = p[0]*cos(joint_variables[0]) + p[1]*sin(joint_variables[0])-0.12
        ct3 = (v**2 + p[2]**2 - 0.26**2 - 0.25**2)/(0.5*0.26)
         
        joint_variables[2] = arccos_d(ct3)  
        joint_variables[1] = (atan2(-0.26*sin(joint_variables[2]), 0.25+0.26*cos(joint_variables[2])) - atan2(p[2], v))  
        joint_variables[3] = ((atan2(a[2], -1*(a[0]*cos(joint_variables[0])+a[1]*sin(joint_variables[0]))) - joint_variables[1] - joint_variables[2]))
        st5 = (a[0]*cos(joint_variables[0])+a[1]*sin(joint_variables[0])-a[2])/(sin(joint_variables[1]+joint_variables[2]+joint_variables[3])+cos(joint_variables[1]+joint_variables[2]+joint_variables[3]))
        ct5 = -1*a[0]*sin(joint_variables[0]) + a[1]*cos(joint_variables[0])
        joint_variables[4] = atan2(st5, ct5)  
        st6 = (-1*o[0]*sin(joint_variables[0])+o[1]*cos(joint_variables[0]))/sin(joint_variables[4])
        ct6 = ((cos(joint_variables[4])*sin(joint_variables[1]+joint_variables[2]+joint_variables[3])*st6)-o[2])/cos(joint_variables[1]+joint_variables[2]+joint_variables[3])
        joint_variables[5] = atan2(st6, ct6)  
        while((joint_variables[3]/rad_perdeg) >= 180 or (joint_variables[3]/rad_perdeg) <=-180):
            if (joint_variables[3]/rad_perdeg) > 0:
                joint_variables[3]= (joint_variables[3]/rad_perdeg - 360)*rad_perdeg
            elif (joint_variables[3]/rad_perdeg) < 0:
                joint_variables[3]= (joint_variables[3]/rad_perdeg + 360)*rad_perdeg
        if arccos_d(ct3) != 10:
            solution += 1
            print ("Solution " + str(solution) + ": ")
            for k in range(6):
                result = check_theta_range(min_angle[k], max_angle[k], k+1, joint_variables[k]/rad_perdeg)
                print("Theta" + str(k+1) + "= " + str(joint_variables[k]/rad_perdeg))
        
        #Solution 7 (theta 3 * -1, theta 4 + 180)
        joint_variables[0] = (atan2(p[1], p[0])  )
        v = p[0]*cos(joint_variables[0]) + p[1]*sin(joint_variables[0])-0.12
        ct3 = (v**2 + p[2]**2 - 0.26**2 - 0.25**2)/(0.5*0.26)
         
        joint_variables[2] = (arccos_d(ct3)  ) * (-1)
        joint_variables[1] = (atan2(-0.26*sin(joint_variables[2]), 0.25+0.26*cos(joint_variables[2])) - atan2(p[2], v))  
        joint_variables[3] = ((atan2(a[2], -1*(a[0]*cos(joint_variables[0])+a[1]*sin(joint_variables[0]))) - joint_variables[1] - joint_variables[2]))
        st5 = (a[0]*cos(joint_variables[0])+a[1]*sin(joint_variables[0])-a[2])/(sin(joint_variables[1]+joint_variables[2]+joint_variables[3])+cos(joint_variables[1]+joint_variables[2]+joint_variables[3]))
        ct5 = -1*a[0]*sin(joint_variables[0]) + a[1]*cos(joint_variables[0])
        joint_variables[4] = atan2(st5, ct5)  
        st6 = (-1*o[0]*sin(joint_variables[0])+o[1]*cos(joint_variables[0]))/sin(joint_variables[4])
        ct6 = ((cos(joint_variables[4])*sin(joint_variables[1]+joint_variables[2]+joint_variables[3])*st6)-o[2])/cos(joint_variables[1]+joint_variables[2]+joint_variables[3])
        joint_variables[5] = atan2(st6, ct6)  
        while((joint_variables[3]/rad_perdeg) >= 180 or (joint_variables[3]/rad_perdeg) <=-180):
            if (joint_variables[3]/rad_perdeg) > 0:
                joint_variables[3]= (joint_variables[3]/rad_perdeg - 360)*rad_perdeg
            elif (joint_variables[3]/rad_perdeg) < 0:
                joint_variables[3]= (joint_variables[3]/rad_perdeg + 360)*rad_perdeg
        if arccos_d(ct3) != 10:
            solution += 1
            print ("Solution " + str(solution) + ": ")
            for k in range(6):
                result = check_theta_range(min_angle[k], max_angle[k], k+1, joint_variables[k]/rad_perdeg)
                print("Theta" + str(k+1) + "= " + str(joint_variables[k]/rad_perdeg))
                

        #Solution 8 (theta 1 + 180, theta 3 * -1, theta4 + 180 )
        joint_variables[0] = (atan2((-1)*p[1], (-1)*p[0])) 
        v = p[0]*cos(joint_variables[0]) + p[1]*sin(joint_variables[0])-0.12
        ct3 = (v**2 + p[2]**2 - 0.26**2 - 0.25**2)/(0.5*0.26)
         
        joint_variables[2] = (arccos_d(ct3)  ) * (-1)
        joint_variables[1] = (atan2(-0.26*sin(joint_variables[2]), 0.25+0.26*cos(joint_variables[2])) - atan2(p[2], v))  
        joint_variables[3] = ((atan2(a[2], -1*(a[0]*cos(joint_variables[0])+a[1]*sin(joint_variables[0]))) - joint_variables[1] - joint_variables[2]))
        st5 = (a[0]*cos(joint_variables[0])+a[1]*sin(joint_variables[0])-a[2])/(sin(joint_variables[1]+joint_variables[2]+joint_variables[3])+cos(joint_variables[1]+joint_variables[2]+joint_variables[3]))
        ct5 = -1*a[0]*sin(joint_variables[0]) + a[1]*cos(joint_variables[0])
        joint_variables[4] = atan2(st5, ct5)  
        st6 = (-1*o[0]*sin(joint_variables[0])+o[1]*cos(joint_variables[0]))/sin(joint_variables[4])
        ct6 = ((cos(joint_variables[4])*sin(joint_variables[1]+joint_variables[2]+joint_variables[3])*st6)-o[2])/cos(joint_variables[1]+joint_variables[2]+joint_variables[3])
        joint_variables[5] = atan2(st6, ct6)  
        while((joint_variables[3]/rad_perdeg) >= 180 or (joint_variables[3]/rad_perdeg) <=-180):
            if (joint_variables[3]/rad_perdeg) > 0:
                joint_variables[3]= (joint_variables[3]/rad_perdeg - 360)*rad_perdeg
            elif (joint_variables[3]/rad_perdeg) < 0:
                joint_variables[3]= (joint_variables[3]/rad_perdeg + 360)*rad_perdeg
        if arccos_d(ct3) != 10:
            solution += 1
            print ("Solution " + str(solution) + ": ")
            for k in range(6):
                result = check_theta_range(min_angle[k], max_angle[k], k+1, joint_variables[k]/rad_perdeg)
                print("Theta" + str(k+1) + "= " + str(joint_variables[k]/rad_perdeg))
        solution = 0 