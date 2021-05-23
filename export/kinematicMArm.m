
function kinematicMArm
p=1/180*pi;
showFlag=1;
begin();
    A= baseA();
   [x,y,z,A,i1,i2,i3]=robot_arm_I(90*p,-90*p,90*p,0,0,0,showFlag)

    [sign_hand, sign_elbow, or_ind]= CalculateI(0,-90*p,90*p,0,0,0)

 [A1,A2,A3,A4,A5,A6, reachFlag]= robot_arm_lin(x,y,z,A,sign_hand, sign_elbow, or_ind,showFlag)
  [Q1, Q2, Q3, Q4, Q5, Q6]= inverceKinematic(x,y,z,A) 
 %[x,y,z,A,i1,i2,i3]=robot_arm_I(0,-90*p,90*p,0,0,0,1)
'OK'
 end    
 