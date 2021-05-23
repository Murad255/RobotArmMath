    
 function simulation
% p0= [0 0 0];
% p1= [5 8 6];
% p2= [10 4 5];
%rm = robot_arm(0,0,0,0,0,0)
view(3)
begin();
simulation3
drawnow
% 'OK'
 end    
 
 function simulation1
    count = 100;
    past=[0 count];
    
    [x,y,z,a]=robot_arm(0,pi/2,0,0,pi/2,pi);
    for k =0:count
        qk=pi*2/count*k; 
    [x,y,z,A]=robot_arm(qk/3,pi/6+qk,qk,0,qk,pi/3);

   % [q1,q2,q3,q4,q5,q6]=forwardKinematic(x,y,z,a,b,c, 1, 1,1,A);
 
       % pause(0.1);
        past(1)=past(1)+1;          
    end

    [x,y,z,A]=robot_arm(0,pi/2,0,0,-pi/4,-pi/2);
    [q1,q2,q3,q4,q5,q6]=forwardKinematicI(x,y,z,A,1, 1,1);
    [q1,q2,q3,q4,q5,q6]=forwardKinematicI(x,y,z,A,-1, 1,1);
     pause(1);
   % robot_arm(q1,q2,q3,q4,q5,q6);
    q1=round(q1*180/pi)
    q2=round(q2*180/pi)
    q3=round(q3*180/pi)
    q4=round(q4*180/pi)
    q5=round(q5*180/pi)
    q6=round(q6*180/pi)
    
%     [attitude,heading,bank]=MatrixToEuler(A);  a=attitude; b=heading;c=pi/2-bank; 
%     attitude=c; bank=(a+pi); heading=(b-pi/2);  A_r=EulerToMatrix(attitude,heading,bank);
 end
   
function simulation2
    count = 150;
    past=[0 count]
    a1=[1;0;0];
    for k =0:count
        qk=pi*2/count*k; 
        A=[cos(qk) -sin(qk) 0; sin(qk) cos(qk) 0; 0 0 1];
        a2=A*a1;
        angle =AngleBetveenVectors(a1,a2);
        title( string(qk*180/pi)+' /'+ string(angle*180/pi))
        pause(0.3);
        past(1)=past(1)+1;          
    end

end

 function simulation3
    count = 100;
    past=[0 count];
    
    A0= [0 0 1; 1 0 0; 0 1 0];
    A = RotateY(A0, pi/4);

    %[x,y,z,A]=robot_arm(0,pi/2,0,0,-pi/4,-pi/2);
    robot_arm_lin(110,0,150,A,1, 1,1);

    for k =0:count
        qk=100*k/count; 
         robot_arm_lin(110+qk,0,150,A,1, 1,1);


       % pause(0.1);
        past(1)=past(1)+1;          
    end

    
 end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  

function [x,y,z,A]=drawRobot(q1,q2,q3,q4,q5,q6) 

 L_01 = 80/10;
 L_11 = 20/10;
 L_2 = 106/10;
 L_40 = 26/10;
 L_41 = 135/10;
 %L_4 = sqrt(L_40 * L_40 + L_41 * L_41);
 L_56 = 80/10;
 
A0=DHmatrix(0,0,0,0);

A1=DHmatrix(q1, L_01,   L_11,   pi/2);
A2= DHmatrix(q2, 0,      L_2,    0);
A3= DHmatrix(q3, 0,      L_40,   pi/2);
A4= DHmatrix(q4, L_41,   0,      -pi/2);
A5= DHmatrix(q5, 0,      0,      pi/2);
A6= DHmatrix(q6, L_56,   0,      0);

Tk=A1*A2*A3*A4*A5*A6;
A_2=A1*A2;
A_3=A1*A2*A3;
A_4=A1*A2*A3*A4;
A_5=A1*A2*A3*A4*A5;
A_6=A1*A2*A3*A4*A5*A6;

M10=[10 0 0 0; 0 10 0 0; 0 0 10 0; 0 0 0 10];
Tk=Tk*M10;
A1=A1*M10;
A_2=A_2*M10;
A_3=A_3*M10;
A_4=A_4*M10;
A_5=A_5*M10;
A_6=A_6*M10;

 drawMatrix(A0,'G');
 drawMatrix(A1,'J_1');
 drawMatrix(A_2,'J_2');
 drawMatrix(A_3,'J_3');
 drawMatrix(A_4,'J_4');
 drawMatrix(A_5,'J_5');
 drawMatrix(A_6,'J_6');

 A_p=A_6*2.5;
A_p(:,4) =[70;0;50;1];
 drawMatrix(A_p,'A_n');
 drawLineMatrix(A0,A1);
 drawLineMatrix(A1,A_2);
 drawLineMatrix(A_2,A_3);
 drawLineMatrix(A_3,A_4);
 drawLineMatrix(A_4,A_5);
 drawLineMatrix(A_5,A_6);
 
r1 = r_draw(1,[q1 q2 q3 q4 q5 q6]);
r2 = r_draw(2,[q1 q2 q3 q4 q5 q6]);
r3 = r_draw(3,[q1 q2 q3 q4 q5 q6]);
r4 = r_draw(4,[q1 q2 q3 q4 q5 q6]);
r5 = r_draw(5,[q1 q2 q3 q4 q5 q6]);
r6 = r_draw(6,[q1 q2 q3 q4 q5 q6]);

%drawLineMatrix(A1,A_6)
tcp= drawPoint(Tk(:,4).');
t=Tk(:,4).';
x=t(1);
y=t(2);
z=t(3);

Tk=Tk/10; Tk(:,4)=[]; Tk(4,:)=[];
A_z=[cos(-pi/2) -sin(-pi/2) 0; sin(-pi/2) cos(-pi/2) 0; 0 0 1];
A_y=[cos(-pi/2) 0 sin(-pi/2); 0 1 0; -sin(-pi/2) 0 cos(-pi/2)];
Tk= Tk*A_z*A_y;
A=A_6/10;A(:,4)=[]; A(4,:)=[];

tcp.Color = 'y';
tcp.Marker='h';
tcp.LineWidth=1;
end


function  [q1,q2,q3,q4,q5,q6]= forwardKinematicI(x,y,z,A, i1, i2, i3) 
% attitude=c;
% bank=(a+pi);
% heading=(b-pi/2);
% A_r=EulerToMatrix(attitude,heading,bank);
% q_z=-pi/2;
% A_z=[cos(q_z) -sin(q_z) 0; sin(q_z) cos(q_z) 0; 0 0 1];
A_p=A;   %A_a*A_b*A_c*A_n;

T_p=[A_p*20 [70;0;50]];
T_p= [T_p; [0 0 0 1]];
drawMatrix(T_p, 'A_p');

P=[x/10; y/10; z/10];
 L_01 = 80/10;
 L_11 = 20/10;
 L_2 = 106/10;
 L_40 = 26/10;
 L_41 = 135/10;
 L_4 = sqrt(L_40 * L_40 + L_41 * L_41);
 L_56 = 80/10;
n =A_p(:,3);
p46=n*L_56;
p04=P-p46;
q1=atan2(p04(2),i1*p04(1));        %TODO L1

p01=L_11*[cos(q1); sin(q1); 0];
p01(3)=L_01;
p14=p04-p01;
np14=norm(p14);
phi=acos((L_2^2+L_4^2-np14^2)/(2*L_2*L_4));
atan2(L_41,L_40)*180/pi;
q3 = i1*((atan2(L_41,L_40))-pi)+phi;  %TODO L

%transform p1 to C1
p14_1=[cos(q1) sin(q1) 0; 0 0 -1; -sin(q1) cos(q1) 0]*p14;
bet_1= atan2(-p14_1(2),p14_1(1));
bet_2=acos((L_2^2-L_4^2+(norm(p14))^2)/(2*L_2*norm(p14)));

q2 =pi/2*(1-i1)+i1*(bet_1+bet_2);               %TODO L
q2g=q2*180/pi;
A01=DHmatrix(q1, L_01,   L_11,   pi/2); A01(4,:)=[];  A01(:,4)=[];
A12= DHmatrix(q2, 0,      L_2,    0);   A12(4,:)=[];  A12(:,4)=[];
A23= DHmatrix(q3, 0,      L_40,   pi/2);A23(4,:)=[];  A23(:,4)=[];

% A01=[cos(q1) -sin(q1) 0; sin(q1) cos(q1) 0; 0 0 1];
% A12=[1 0 0 ; 0 cos(q2) -sin(q2); 0 sin(q2) cos(q2)];
% A23=[1 0 0 ; 0 cos(q3) -sin(q3); 0 sin(q3) cos(q3)];
A03=A01*A12*A23;
A02=A01*A12;

p12=A02*[L_2;0 ; 0];
p23=A03*[L_40;0 ; 0];

p34=p14-p12-p23;
z3=p34/norm(p34);
q5=i1*acos(z3.'*n);

y3=-1*A01(:,3);
c=cross(n,y3)/norm(cross(n,y3));% todo
c=cross(z3,c); c=c/norm(c);
q4=(pi - acos(c.'*(y3*(-1))))*i1;     %TODO L

A34= DHmatrix(q4, L_41,   0,      -pi/2);   A34(4,:)=[];  A34(:,4)=[];
A45= DHmatrix(q5, 0,      0,      pi/2);    A45(4,:)=[];  A45(:,4)=[];
A05=A01*A12*A23*A34*A45;

q6=AngleBetveenVectors(A05(:,1),A_p(:,1));
sigm=AngleBetveenVectors3(A05(:,2),A_p(:,1),n);
if(sigm<=pi/2)
    q6=abs(q6)*i1;                  %TODO L
elseif(sigm>pi/2)
    q6=-1*abs(q6)*i1;
end
q1=atan2(p04(2),i1*p04(1));   
q2 =pi/2*(1-i1)+i1*q2; 
q3 = i1*((atan2(L_41,L_40))-pi)+phi; 
end
function  [q1,q2,q3,q4,q5,q6]= forwardKinematic(x,y,z,A) 
% attitude=c;
% bank=(a+pi);
% heading=(b-pi/2);
% A_r=EulerToMatrix(attitude,heading,bank);
% q_z=-pi/2;
% A_z=[cos(q_z) -sin(q_z) 0; sin(q_z) cos(q_z) 0; 0 0 1];
A_p=A;   %A_a*A_b*A_c*A_n;

T_p=[A_p*20 [70;0;50]];
T_p= [T_p; [0 0 0 1]];
drawMatrix(T_p, 'A_p');

P=[x/10; y/10; z/10];
 L_01 = 80/10;
 L_11 = 20/10;
 L_2 = 106/10;
 L_40 = 26/10;
 L_41 = 135/10;
 L_4 = sqrt(L_40 * L_40 + L_41 * L_41);
 L_56 = 80/10;
n =A_p(:,3);
p46=n*L_56;
p04=P-p46;
q1=atan2(p04(2),p04(1));        %TODO L1

p01=L_11*[cos(q1); sin(q1); 0];
p01(3)=L_01;
p14=p04-p01;
np14=norm(p14);
phi=acos((L_2^2+L_4^2-np14^2)/(2*L_2*L_4));
atan2(L_41,L_40)*180/pi;
q3 = (phi+atan2(L_41,L_40))-pi;  %TODO L

%transform p1 to C1
p14_1=[cos(q1) sin(q1) 0; 0 0 -1; -sin(q1) cos(q1) 0]*p14;
bet_1= atan2(-p14_1(2),p14_1(1));
bet_2=acos((L_2^2-L_4^2+(norm(p14))^2)/(2*L_2*norm(p14)));
q2 =(bet_1+bet_2);               %TODO L

A01=DHmatrix(q1, L_01,   L_11,   pi/2); A01(4,:)=[];  A01(:,4)=[];
A12= DHmatrix(q2, 0,      L_2,    0);   A12(4,:)=[];  A12(:,4)=[];
A23= DHmatrix(q3, 0,      L_40,   pi/2);A23(4,:)=[];  A23(:,4)=[];

% A01=[cos(q1) -sin(q1) 0; sin(q1) cos(q1) 0; 0 0 1];
% A12=[1 0 0 ; 0 cos(q2) -sin(q2); 0 sin(q2) cos(q2)];
% A23=[1 0 0 ; 0 cos(q3) -sin(q3); 0 sin(q3) cos(q3)];
A03=A01*A12*A23;
A02=A01*A12;

p12=A02*[L_2;0 ; 0];
p23=A03*[L_40;0 ; 0];

p34=p14-p12-p23;
z3=p34/norm(p34);
q5=acos(z3.'*n);

y3=-1*A01(:,3);
c=cross(n,y3)/norm(cross(n,y3));% todo
c=cross(z3,c); c=c/norm(c);
q4=pi - acos(c.'*(y3*(-1)));     %TODO L

A34= DHmatrix(q4, L_41,   0,      -pi/2);   A34(4,:)=[];  A34(:,4)=[];
A45= DHmatrix(q5, 0,      0,      pi/2);    A45(4,:)=[];  A45(:,4)=[];
A05=A01*A12*A23*A34*A45;

q6=AngleBetveenVectors(A05(:,1),A_p(:,1));
sigm=AngleBetveenVectors3(A05(:,2),A_p(:,1),n);
if(sigm<=pi/2)
    q6=abs(q6);                  %TODO L
elseif(sigm>pi/2)
    q6=-1*abs(q6);
end

end

function [i1,i2,i3]= CalculateI(q1,q2,q3,q4,q5,q6,x)
	 L_01 = 80/10;
     L_11 = 20/10;
     L_2 = 106/10;
     L_40 = 26/10;
     L_41 = 135/10;
     %L_4 = sqrt(L_40 * L_40 + L_41 * L_41);
     L_56 = 80/10;

%     A1= DHmatrix(0, L_01,   L_11,   pi/2);
%     A2= DHmatrix(q2, 0,      L_2,    0);
%     A3= DHmatrix(q3, 0,      L_40,   pi/2);
%     A4= DHmatrix(0, L_41,   0,      -pi/2);
%     Tk= A1*A2*A3*A4;

    i1= 1;
    i2= 1;
    i3=1;

    if (x < 0)		
        i1= -1;
    end
		q3a = (pi/2 - atan2(L_41, L_40))*180/pi;
		if ((q3 + 90 - q3a) * i1 < 0)
			i2 = -1;
		end
		if (q5 < 0)
			i3 = -1;
        end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [r_i]=r(i,q) 
r_0 = [0;0;0;1];
 L_01 = 80;
 L_11 = 20;
 L_2 = 106;
 L_40 = 26;
 L_41 = 135;
 %L_4 = sqrt(L_40 * L_40 + L_41 * L_41);
 L_56 = 80;
 
 if(i==0)
    r_i = r_0;
 elseif(i==1)
    r_i =  getA(1,q)*[-10; -20; 0 ; 1];
 elseif(i==2)
    r_i =  getA2(1,2,q)*[- L_2/2; 0; 0 ; 1];
 elseif(i==3)
    r_i = getA2(1,3,q)*[- L_40/2; 0; 0 ; 1];
 elseif(i==4)
    r_i = getA2(1,4,q)*[0;  L_41/2; 0 ; 1];
 elseif(i==5)
   r_i = getA2(1,5,q)*[0; 0;  L_56/2 ; 1];
 elseif(i==6)
   r_i = getA2(1,6,q)*[0; 0; - 5 ; 1];          
 end
 
end

function [r_i]=r_draw(i,q) 
r_i = r(i,q);
 
pr1=drawPoint(r_i);
pr1.Marker='h';
pr1.LineWidth=1;
text(r_i(1),r_i(2),r_i(3),'m'+string(i));
end

function  A = getA2(i,j,q) 
A=getA(i,q);
for k = (i+1):j
    A=A*getA(k,q);
end

end

function  A = getA(i,q) 
 L_01 = 80;
 L_11 = 20;
 L_2 = 106;
 L_40 = 26;
 L_41 = 135;
 %L_4 = sqrt(L_40 * L_40 + L_41 * L_41);
 L_56 = 80;
  
if(i==0)
    A=DHmatrix(0, 0,   0,   0);
elseif(i==1)
    A=DHmatrix(q(1), L_01,   L_11,   pi/2);
elseif(i==2)
    A= DHmatrix(q(2), 0,      L_2,    0);
elseif(i==3)
    A= DHmatrix(q(3), 0,      L_40,   pi/2);
elseif(i==4)
    A= DHmatrix(q(4), L_41,   0,      -pi/2);
elseif(i==5)
    A= DHmatrix(q(5), 0,      0,      pi/2);
elseif(i==6)
    A= DHmatrix(q(6), L_56,   0,      0);
end

end

function A=EulerToMatrix(attitude,heading,bank)
sa = sin(attitude);
ca = cos(attitude);
sb = sin(bank);
cb = cos(bank);
sh = sin(heading);
ch = cos(heading);
A=[
    ch*ca   -ch*sa*cb+sh*sb ch*sa*sb + sh*cb;...
    sa      ca*cb           -ca*sb; ...
    -sh*ca 	sh*sa*cb+ch*sb  -sh*sa*sb + ch*cb
]; 
% A_n=[0 0 1;0 -1 0;1 0 0];
% A_c=[cos(c) -sin(c) 0; sin(c) cos(c) 0; 0 0 1];
% A_b=[cos(b) 0 sin(b); 0 1 0; -sin(b) 0 cos(b)];
% A_a=[1 0 0 ; 0 cos(a) -sin(a); 0 sin(a) cos(a)];
end 

function [attitude,heading,bank]=MatrixToEuler(A)
    if (A(2,1) > 0.998) 
        heading = atan2(A(1,3),A(3,3));
        attitude = pi/2;
        bank = 0;

    elseif  (A(2,1) < -0.998) 
        heading =atan2(A(1,3),A(3,3));
        attitude = -pi/2;
        bank = 0;

    else 
        heading = atan2(-A(3,1),A(1,1));
        bank = atan2(-A(2,3),A(2,2));
        attitude = asin(A(2,1));
    end
end

function [a]= AngleBetveenVectors(V1,V2)
     V1n =V1/norm(V1);
     V2n =V2/norm(V2);
a=acos(V1n'*V2n);
end

function [angle]= AngleBetveenVectors3( V1,  V2,  N)

     V1n =V1/norm(V1);
     V2n =V2/norm(V2);

    angle = atan2(dot(N, cross(V1n, V2n)), dot(V1n, V2n));
    %double n = (V1n.x * V2n.x + V1n.y * V2n.y + V1n.z * V2n.z) >= 0 ? 1 : -1;
    %a= -1 * angle;%n * angle;
end

function  [x,y,z,A]=robot_arm(q1,q2,q3,q4,q5,q6)
    %ROBOT_ARM Summary of this function goes here
    %   Detailed explanation goes here.
    begin() 
    [x,y,z,A]= drawRobot(q1,q2,q3,q4,q5,q6);
    [i1,i2,i3] = CalculateI(q1,q2,q3,q4,q5,q6,x);
    [a,b,c]=MatrixToEuler(A);
    drawnow
    title('The graphic of robot-arm| I1 = '+string(i1)+'; I2 = '+string(i2)+'; I3 = '+string(i3));
    xlabel('X = '+string(round(x))+' C = '+string(round(c*180/pi)));
    ylabel('Y = '+string(round(y))+' B = '+string(round(b*180/pi)));
    zlabel('Z = '+string(round(z))+' A = '+string(round(a*180/pi)));
end

function  begin()
    cla 
    %axes('Xlim',[-5 15], 'Ylim',[-5 15], 'Zlim',[-5 15]); 
   % view(3)
    axis equal; grid on; hold on; 
    drawCoordSystem(15);
end

function drawLineMatrix(An,Ak) 
pn= An(:,4).';
pk= Ak(:,4).';
pn(4)=[];
pk(4)=[];

 drawLine(pn,pk);
end

function drawMatrix(A,str) 
px=A(:,1).';
py=A(:,2).';
pz=A(:,3).';
p= A(:,4).';
px(4)=[];
py(4)=[];
pz(4)=[];
p(4)=[];

px= transferPoint(px,p);
py= transferPoint(py,p);
pz= transferPoint(pz,p);

lin_x= drawLine(p,px);
lin_y= drawLine(p,py);
lin_z= drawLine(p,pz);

lin_x.Color = 'r';
lin_y.Color = 'g';
lin_z.Color = 'b';

text(p(1),p(2),p(3),str);
end

function lin=drawLine(pln,plk) 
xl = linspace(pln(1),plk(1));
yl = linspace(pln(2),plk(2));
zl = linspace(pln(3),plk(3));

lin=line(xl,yl,zl);
end

function drawCoordSystem(size) 
o=[0 0 0];
ox= [size 0 0];
oy= [0 size 0];
oz= [0 0 size];

lin_x=drawLine(o,ox);
lin_y=drawLine(o,oy);
lin_z=drawLine(o,oz);

lin_x.Color = 'r';
lin_y.Color = 'g';
lin_z.Color = 'b';

end

function lin=drawPoint(p) 
lin=line(p(1),p(2),p(3));
end

function matrix=DHmatrix(Q,d,a,alf)
m1=[cos(Q) -cos(alf)*sin(Q)     sin(alf)*sin(Q)    a*cos(Q)];
m2=[sin(Q)  cos(alf)*cos(Q)     -sin(alf)*cos(Q)    a*sin(Q)];
m3=[0       sin(alf)            cos(alf)            d];
m4=[0       0                   0                   1];
matrix= [m1;m2;m3;m4];
end

function p=transferPoint(p,pt)
pm=[1 0 0 p(1); 0 1 0 p(2); 0 0 1 p(3); 0 0 0 1];
ptm=[1 0 0 pt(1); 0 1 0 pt(2); 0 0 1 pt(3); 0 0 0 1];
p=pm*ptm;
p=p(:,4).';
p(4)=[];
end

function T=RotateX(A,angle)
  
AX= [1 0 0; ...
     0 cos(angle) -sin(angle); ...
     0 sin(angle) cos(angle)];
   T=AX*A;  
end

function T=RotateY(A,angle)
  
AY= [cos(angle) 0 sin(angle); ...
     0 1 0; ...
     -sin(angle) 0 cos(angle)];
   T=AY*A;
end

function T=RotateZ(A,angle)
AZ= [cos(angle) -sin(angle) 0; ...
     sin(angle) cos(angle)  0; ...
     0 0 1];
   T=AZ*A;  
   
end
