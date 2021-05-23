%рассчёт флагов
function [sign_hand, sign_elbow, or_ind]= CalculateI(A1,A2,A3,A4,A5,A6)
Q=[A1;A2;A3;A4;A5;A6]
R=[0;0;0;1];
d=[0;150;0;430;0;60];
a=[0;430;-20;0;0;0];
alfa=[-pi/2; 0; pi/2;-pi/2;pi/2;0];

H=getA2(1,6,Q);
R=H*R;
nx= H( 1,1 ); sx= H(1 ,2 ); ax= H( 1, 3); px= H(1 , 4);
ny= H( 2,1 ); sy= H(2 ,2 ); ay= H( 2, 3); py= H(2 , 4);
nz= H( 3,1 ); sz= H(3 ,2 ); az= H( 3, 3); pz= H(3 , 4);

p4x = px - d(6)*ax;
p4y = py - d(6)*ay;
p4z = pz - d(6)*az;

sign_hand_t = -d(4)*sin(Q(2)+Q(3))-a(3)*cos(Q(2)+Q(3))-a(2)*cos(Q(2));
if(sign_hand_t<0)   sign_hand = -1; 
else                sign_hand = 1;
end


sign_elbow_t =  sign_hand*(d(4)*cos(Q(3))-a(3)*sin(Q(3)) ) ;
if(sign_elbow_t<0)  sign_elbow = -1; 
else                sign_elbow = 1;
end  



%indicator wrist calculation
Z3 = [ cos(Q(1))*sin(Q(2)+Q(3)); sin(Q(1))*sin(Q(2)+Q(3)); cos(Q(2)+Q(3))];
vector_a = [ax;ay;az];
z3vector_a = cross(Z3,vector_a); %todo
z3vector_a_norm = sqrt(z3vector_a(1)^2+z3vector_a(2)^2+z3vector_a(3)^2);
Z4 = [z3vector_a(1)/z3vector_a_norm; z3vector_a(2)/z3vector_a_norm; z3vector_a(3)/z3vector_a_norm];
vector_s = [sx;sy;sz];
vector_n = [nx;ny;nz];
z4p = [ -cos(Q(4))*sin(Q(1))-cos(Q(2)+Q(3))*cos(Q(1))*sin(Q(4));...
         cos(Q(1))*cos(Q(4))-cos(Q(2)+Q(3))*sin(Q(1))*sin(Q(4));...
         sin(Q(2)+Q(3))*sin(Q(4))
         ];
VSZ = dot(vector_s,Z4);
VNZ = dot(vector_n,Z4);

if(VSZ>0)wrist_s=1;  
else wrist_s=-1;
end
if(VSZ>0)wrist_n=1;  
else wrist_n=-1;
end
if(VSZ==0)wrist=wrist_n; 
else wrist=wrist_s;
end   

VSZp =  dot(vector_s,z4p);
VNZp =  dot(vector_n,z4p);

sign_s = -1;
if(VSZp>0)sign_s=1;   end   
sign_n = -1;
if(VSZ>0)sign_n=1;   end   
sign = sign_s;
if(VSZ==0)sign=sign_n;   end   

or_ind = sign* wrist;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%возвращает произведение матриц преобразования от i-той до j-той
function  A = getA2(i,j,q) 
A=getA(i,q);
for k = (i+1):j
    A=A*getA(k,q);
end

end

%возвращает i-тую матрицу преобразования
function  A = getA(i,q) 

alfa=[-pi/2; 0; pi/2;-pi/2;pi/2;0];

if(i==0)
    A=DHmatrix(0, 0,   0,   0);
elseif(i==1)
    A=DHmatrix(q(1),d(1),a(1),alfa(1));
elseif(i==2)
    A=DHmatrix(q(2),d(2),a(2),alfa(2));
elseif(i==3)
    A=DHmatrix(q(3),d(3),a(3),alfa(3));
elseif(i==4)
    A=DHmatrix(q(4),d(4),a(4),alfa(4));
elseif(i==5)
    A=DHmatrix(q(5),d(5),a(5),alfa(5));
elseif(i==6)
    A=DHmatrix(q(6),d(6),a(6),alfa(6));
end

end

function  x = d(i) 
d=[0;0;0;153;0;57];
x= d(i);
end

function  x = a(i) 
a=[0;105;-35;0;0;0];
x= a(i);
end

%построение матрицы по параметрам Денавита–Хартенберга
function matrix=DHmatrix(Q,d,a,alf)
m1=[cos(Q) -cos(alf)*sin(Q)     sin(alf)*sin(Q)    a*cos(Q)];
m2=[sin(Q)  cos(alf)*cos(Q)     -sin(alf)*cos(Q)    a*sin(Q)];
m3=[0       sin(alf)            cos(alf)            d];
m4=[0       0                   0                   1];
matrix= [m1;m2;m3;m4];
end



 