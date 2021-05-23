function Kinematic
q=[30;90;90;20;-45;60]
d=[0;150;0;430;0;60];
a=[0;430;-20;0;0;0];
alfa=[-pi/2; 0; pi/2;-pi/2;pi/2;0];

Q=q*pi/180; Q(3)=pi/2+q(3)*pi/180;

A0=getA(0,Q);
A1=getA(1,Q);
A2=getA(2,Q);
A3=getA(3,Q);
A4=getA(4,Q);
A5=getA(5,Q);
A6=getA(6,Q);

H=A1*A2*A3*A4*A5*A6;
H1=A1*A2*A3;
H2=A4*A5*A6;
R=[0;0;0;1];

R=H*R;
Q=Q*180/pi;
nx= H( 1,1 ); sx= H(1 ,2 ); ax= H( 1, 3); px= H(1 , 4);
ny= H( 2,1 ); sy= H(2 ,2 ); ay= H( 2, 3); py= H(2 , 4);
nz= H( 3,1 ); sz= H(3 ,2 ); az= H( 3, 3); pz= H(3 , 4);

%inverse orientation task
ori_T = atan22(nz,-sz)*180/pi ;
ori_A = atan22(az, -nz*cos(ori_T*pi/180)+sz*sin(ori_T*pi/180))*180/pi;
ori_O = atan22(ny*sin(ori_T*pi/180)+sy*cos(ori_T*pi/180), -nx*sin(ori_T*pi/180)-sx*cos(ori_T*pi/180) )*180/pi; 

ori_Matrix = [  cos(ori_A * pi/180)*cos(ori_O * pi/180)*cos(ori_T * pi/180)-sin(ori_O * pi/180)*sin(ori_T * pi/180)...
                -sin(ori_O * pi/180)*cos(ori_T * pi/180)-cos(ori_A * pi/180)*cos(ori_O * pi/180)*sin(ori_T * pi/180)...
                sin(ori_A * pi/180)*cos(ori_O * pi/180);...
                
                cos(ori_O * pi/180)*sin(ori_T * pi/180)+cos(ori_A * pi/180)*sin(ori_O * pi/180)*cos(ori_T * pi/180)...
                cos(ori_O * pi/180)*cos(ori_T * pi/180)-cos(ori_A * pi/180)*sin(ori_O * pi/180)*sin(ori_T * pi/180)...
                sin(ori_A * pi/180)*sin(ori_O * pi/180);...
                
                -sin(ori_A * pi/180)*cos(ori_T * pi/180)...
                sin(ori_A * pi/180)*sin(ori_T * pi/180)...
                cos(ori_A * pi/180)...
                ];
            
%direct orientation task
ori_A=0;
ori_T=0;
ori_O =0;
nxp=cos(ori_A * pi/180)*cos(ori_O * pi/180)*cos(ori_T * pi/180)-sin(ori_O * pi/180)*sin(ori_T * pi/180);
sxp=-sin(ori_O * pi/180)*cos(ori_T * pi/180)-cos(ori_A * pi/180)*cos(ori_O * pi/180)*sin(ori_T * pi/180); 
axp=sin(ori_A * pi/180)*cos(ori_O * pi/180); 

nyp= cos(ori_O * pi/180)*sin(ori_T * pi/180)+cos(ori_A * pi/180)*sin(ori_O * pi/180)*cos(ori_T * pi/180); 
syp=cos(ori_O * pi/180)*cos(ori_T * pi/180)-cos(ori_A * pi/180)*sin(ori_O * pi/180)*sin(ori_T * pi/180); 
ayp=sin(ori_A * pi/180)*sin(ori_O * pi/180); 

nzp=-sin(ori_A * pi/180)*cos(ori_T * pi/180); 
szp= sin(ori_A * pi/180)*sin(ori_T * pi/180); 
azp=cos(ori_A * pi/180); 

%Preparation to inverse kinematic solution


nx = ((cosp(Q(2)+Q(3))*cosp(Q(1))*cosp(Q(4))-sinp(Q(1))*sinp(Q(4)))*cosp(Q(5))+(-sinp(Q(2)+Q(3))*cosp(Q(1)))*sinp(Q(5)))*cosp(Q(6))+((-cosp(Q(4))*sinp(Q(1))-cosp(Q(2)+Q(3))*cosp(Q(1))*sinp(Q(4)))*sinp(Q(6)));
ny = ((cosp(Q(1))*sinp(Q(4))+cosp(Q(2)+Q(3))*cosp(Q(4))*sinp(Q(1)))*cosp(Q(5))+(-sinp(Q(2)+Q(3))*sinp(Q(1)))*sinp(Q(5)))*cosp(Q(6))+((cosp(Q(1))*cosp(Q(4))-cosp(Q(2)+Q(3))*sinp(Q(1))*sinp(Q(4)))*sinp(Q(6)));
nz = ((-sinp(Q(2)+Q(3))*cosp(Q(4)))*cosp(Q(5))+(-cosp(Q(2)+Q(3)))*sinp(Q(5)) )*cosp(Q(6))+(sinp(Q(2)+Q(3))*sinp(Q(4)))*sinp(Q(6));

sx = (-cosp(Q(4))*sinp(Q(1))-cosp(Q(2)+Q(3))*cosp(Q(1))*sinp(Q(4)))*cosp(Q(6))-((cosp(Q(2)+Q(3))*cosp(Q(1))*cosp(Q(4))-sinp(Q(1))*sinp(Q(4)))*cosp(Q(5))+(-sinp(Q(2)+Q(3))*cosp(Q(1)))*sinp(Q(5)) )*sinp(Q(6));
sy = (cosp(Q(1))*cosp(Q(4))-cosp(Q(2)+Q(3)) *sinp(Q(1))*sinp(Q(4)))*cosp(Q(6))-((cosp(Q(1))*sinp(Q(4))*cosp(Q(4))+cosp(Q(2)+Q(3))*cosp(Q(4)))*cosp(Q(5))+(-sinp(Q(2)+Q(3))*sinp(Q(1)))*sinp(Q(5)) )*sinp(Q(6));
sz = (sinp(Q(2)+Q(3))*sinp(Q(4)) )*cosp(Q(6)) -((-sinp(Q(2)+Q(3))*cosp(Q(4))) *cosp(Q(5)) +(-cosp(Q(2)+Q(3)) ))*sinp(Q(5))*sinp(Q(6));

ax = ((cosp(Q(2)+Q(3))*cosp(Q(1))*cosp(Q(4))-sinp(Q(1))*sinp(Q(4)))*sinp(Q(5))-(-sinp(Q(2)+Q(3))*cosp(Q(1)))*cosp(Q(5))  );
ay = ((cosp(Q(1))*sinp(Q(4))+cosp(Q(2)+Q(3))*cosp(Q(4))*sinp(Q(1)))*sinp(Q(5))-(-sinp(Q(2)+Q(3))*sinp(Q(1)))*cosp(Q(5))  );
az = (((-sinp(Q(2)+Q(3))*cosp(Q(4)))*sinp(Q(5)))-(-cosp(Q(2)+Q(3)))*cosp(Q(5)) );
 
px = (cosp(Q(1))*cosp(Q(2))*a(2)-sinp(Q(1))*d(2)+cosp(Q(2)+Q(3))*cosp(Q(1))*a(3)+sinp(Q(2)+Q(3))*cosp(Q(1))*d(4))+((cosp(Q(2)+Q(3))*cosp(Q(1))*cosp(Q(4))-sinp(Q(1))*sinp(Q(4)))*sinp(Q(5))-(-sinp(Q(2)+Q(3))*cosp(Q(1)))*cosp(Q(5)))*d(6);
py = (cosp(Q(1))*d(2)+cosp(Q(2))*sinp(Q(1))*a(2)+cosp(Q(2)+Q(3))*sinp(Q(1))*a(3)+sinp(Q(2)+Q(3))*sinp(Q(1))*d(4))+((cosp(Q(1))*sinp(Q(4))+cosp(Q(2)+Q(3))*cosp(Q(4))*sinp(Q(1)))*sinp(Q(5))-(-sinp(Q(2)+Q(3))*sinp(Q(1)))*cosp(Q(5)))*d(6);
pz = (cosp(Q(2)+Q(3))*d(4)-sinp(Q(2)+Q(3))*a(3)-sinp(Q(2))*(a(2)+d(1)))+(((-sinp(Q(2)+Q(3))*cosp(Q(4)))*sinp(Q(5)))-(-cosp(Q(2)+Q(3)))*cosp(Q(5)) )*d(6);

temp_A = [nx sx ax px; ny sy ay py; nz sz az pz ];

p4x = px - d(6)*ax;
p4y = py - d(6)*ay;
p4z = pz - d(6)*az;
p4 = [p4x; p4y; p4z];

% Inverse algebraic kinematic solution
% indicator hand calculation:
sign_hand = 1;
sign_hand_t = -d(4)*sinp(Q(2)+Q(3))-a(3)*cosp(Q(2)+Q(3))-a(2)*cosp(Q(2));
if(sign_hand_t<0) sign_hand = -1; end  

% 1
Q1 = atan22(-sign_hand*p4x*sqrt(p4x^2+p4y^2-(d(2))^2)+p4y*d(2), -sign_hand*p4y*sqrt(p4x^2+p4y^2-(d(2))^2)-p4x*d(2) )*180/pi;
% indicator elbow calculation:
sign_elbow = 1;
sign_elbow_t =  sign_hand*(d(4)*cosp(Q(3))-a(3)*sinp(Q(3)) ) ;
if(sign_elbow_t<0) sign_elbow = -1; end  

% 2
sin_a_Q2 = -p4z/sqrt(p4x^2+p4y^2-(d(2))^2+p4z^2);
cos_a_Q2 = -sign_hand*sqrt(p4x^2+p4y^2-(d(2))^2)/sqrt(p4x^2+p4y^2-(d(2))^2+p4z^2);
cos_b_Q2 = (a(2)^2+p4x^2++p4y^2-d(2)^2+p4z^2 - (a(3)^2+d(4)^2))/(2*a(2)*sqrt(p4x^2++p4y^2-d(2)^2+p4z^2));
sin_b_Q2 = sqrt(1-cos_b_Q2^2);

sin_Q2 = sin_a_Q2*cos_b_Q2 +(sign_elbow*sign_hand)*cos_a_Q2*sin_b_Q2 ;
cos_Q2 = cos_a_Q2*cos_b_Q2 -(sign_elbow*sign_hand)*sin_a_Q2*sin_b_Q2 ;
Q2 = atan22(cos_Q2,sin_Q2)*180/pi;

% Q3
sin_b_Q3 = abs(a(3))/sqrt(a(3)^2+d(4)^2);
cos_b_Q3 = d(4)/sqrt(a(3)^2+d(4)^2);
cos_fi_Q3= (a(2)^2+a(3)^2+d(4)^2-(p4x^2++p4y^2-d(2)^2+p4z^2))/(2*a(2)*sqrt(a(3)^2+d(4)^2 ));
sin_fi_Q3= sign_elbow*sign_hand*sqrt(1-cos_fi_Q3^2);

sin_Q3 = -sin_fi_Q3*cos_b_Q3-cos_fi_Q3*sin_b_Q3 ;
cos_Q3 = -cos_fi_Q3*cos_b_Q3+sin_fi_Q3*sin_b_Q3 ;

Q3 = atan22(cos_Q3,sin_Q3)*180/pi;

%indicator wrist calculation
Z3 = [ cosp(Q(1))*sinp(Q(2)+Q(3)); sinp(Q(1))*sinp(Q(2)+Q(3)); cosp(Q(2)+Q(3))];
vector_a = [ax;ay;az];
z3vector_a = cross(Z3,vector_a); %todo
z3vector_a_norm = sqrt(z3vector_a(1)^2+z3vector_a(2)^2+z3vector_a(3)^2);
Z4 = [z3vector_a(1)/z3vector_a_norm; z3vector_a(2)/z3vector_a_norm; z3vector_a(3)/z3vector_a_norm];
vector_s = [sx;sy;sz];
vector_n = [nx;ny;nz];
z4p = [ -cosp(Q(4))*sinp(Q(1))-cosp(Q(2)+Q(3))*cosp(Q(1))*sinp(Q(4));...
         cosp(Q(1))*cosp(Q(4))-cosp(Q(2)+Q(3))*sinp(Q(1))*sinp(Q(4));...
         sinp(Q(2)+Q(3))*sinp(Q(4))
         ];
VSZ = dot(vector_s,Z4);
VNZ = dot(vector_n,Z4);
wrist_s=-1;
if(VSZ>0)wrist_s=1;   end   
wrist_n=-1;
if(VSZ>0)wrist_n=1;   end   
wrist=wrist_s;
if(VSZ==0)wrist=wrist_n;   end   

VSZp =  dot(vector_s,z4p);
VNZp =  dot(vector_n,z4p);

sign_s = -1;
if(VSZp>0)sign_s=1;   end   
sign_n = -1;
if(VSZ>0)sign_n=1;   end   
sign = sign_s;
if(VSZ==0)sign=sign_n;   end   

or_ind = sign* wrist;

% 4
sin_Q4 = or_ind * (-sinp(Q(1))*ax+cosp(Q(1))*ay);
cos_Q4 = or_ind * (cosp(Q(1))*cosp(Q(2)+Q(3))*ax + sinp(Q(1))*cosp(Q(2)+Q(3))*ay-sinp(Q(2)+Q(3))*az );
Q4 = atan22(cos_Q4,sin_Q4)*180/pi;

% 5
sin_Q5 = (cosp(Q(1))*cosp(Q(4))*cosp(Q(2)+Q(3))-sinp(Q(1))*sinp(Q(4)))*ax +(cosp(Q(1))*sinp(Q(4))+cosp(Q(4))*sinp(Q(1))*cosp(Q(2)+Q(3)))*ay-cosp(Q(4))*sinp(Q(2)+Q(3))*az ;
cos_Q5 = (cosp(Q(1))*sinp(Q(2)+Q(3))*ax + sinp(Q(1))*sinp(Q(2)+Q(3))*ay)+ cosp(Q(2)+Q(3))*az;
Q5 = atan22(cos_Q5, sin_Q5) *180/pi;

% 6
sin_Q6 = ((-cosp(Q(1))*sinp(Q(4))*cosp(Q(2)+Q(3))-sinp(Q(1))*cosp(Q(4)))*nx +(cosp(Q(1))*cosp(Q(4))- sinp(Q(4))*sinp(Q(1))*cosp(Q(2)+Q(3)))*ny)+sinp(Q(4))*sinp(Q(2)+Q(3))*nz ;
cos_Q6 = ((cosp(Q(2)+Q(3))*cosp(Q(1))*cosp(Q(4))-sinp(Q(1))*sinp(Q(4)))*cosp(Q(5))+(-sinp(Q(2)+Q(3))*cosp(Q(1)))*sinp(Q(5)))*nx+(((cosp(Q(1))*sinp(Q(4))+cosp(Q(2)+Q(3))*cosp(Q(4))*sinp(Q(1)))*cosp(Q(5))+(-sinp(Q(2)+Q(3))*sinp(Q(1)))*sinp(Q(5))))*ny+ ((-sinp(Q(2)+Q(3))*cosp(Q(4)))*cosp(Q(5))+(-cosp(Q(2)+Q(3)))*sinp(Q(5)))*nz;
    
Q6 = atan22(cos_Q6,sin_Q6)*180/pi;

test_Q = [Q1 Q2 Q3 Q4 Q5 Q6]


'OK'
 end    
 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  

% тот же atan2, только меняет местами аргументы
function fi = atan22(x,y) 
 fi=atan2(y,x);
end

% функция cos с переводом входного параметра из углов в радианы
function fi = cosp(a) 
 fi=cos(a*pi/180);
end

% функция sin с переводом входного параметра из углов в радианы
function fi = sinp(a) 
 fi=sin(a*pi/180);
end

%возвращает произведение матриц преобразования от i-той до j-той
function  A = getA2(i,j,q) 
A=getA(i,q);
for k = (i+1):j
    A=A*getA(k,q);
end

end

%возвращает i-тую матрицу преобразования
function  A = getA(i,q) 
d=[0;150;0;430;0;60];
a=[0;430;-20;0;0;0];
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

%построение матрицы по параметрам Денавита–Хартенберга
function matrix=DHmatrix(Q,d,a,alf)
m1=[cos(Q) -cos(alf)*sin(Q)     sin(alf)*sin(Q)    a*cos(Q)];
m2=[sin(Q)  cos(alf)*cos(Q)     -sin(alf)*cos(Q)    a*sin(Q)];
m3=[0       sin(alf)            cos(alf)            d];
m4=[0       0                   0                   1];
matrix= [m1;m2;m3;m4];
end


