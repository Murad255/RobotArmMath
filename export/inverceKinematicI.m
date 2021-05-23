
%ОЗК
function  [Q1, Q2, Q3, Q4, Q5, Q6]= inverceKinematicI(x,y,z,A, sign_hand, sign_elbow, or_ind) 

H = A;
H(:,4) = [x; y; z];
H(4,:) = [0 0 0 1];

R=[0;0;0;1];
R=H*R;

%d=[0;150;0;430;0;60];
%a=[0;430;-20;0;0;0];
alfa=[-pi/2; 0; pi/2;-pi/2;pi/2;0];

nx= H( 1,1 ); sx= H(1 ,2 ); ax= H( 1, 3); px= H(1 , 4);
ny= H( 2,1 ); sy= H(2 ,2 ); ay= H( 2, 3); py= H(2 , 4);
nz= H( 3,1 ); sz= H(3 ,2 ); az= H( 3, 3); pz= H(3 , 4);

%inverse orientation task
% ori_T = atan22(nz,-sz)*180/pi ;
% ori_A = atan22(az, -nz*cos(ori_T*pi/180)+sz*sin(ori_T*pi/180))*180/pi;
% ori_O = atan22(ny*sin(ori_T*pi/180)+sy*cos(ori_T*pi/180), -nx*sin(ori_T*pi/180)-sx*cos(ori_T*pi/180) )*180/pi; 

% ori_Matrix = [  cos(ori_A * pi/180)*cos(ori_O * pi/180)*cos(ori_T * pi/180)-sin(ori_O * pi/180)*sin(ori_T * pi/180)...
%                 -sin(ori_O * pi/180)*cos(ori_T * pi/180)-cos(ori_A * pi/180)*cos(ori_O * pi/180)*sin(ori_T * pi/180)...
%                 sin(ori_A * pi/180)*cos(ori_O * pi/180);...
%                 
%                 cos(ori_O * pi/180)*sin(ori_T * pi/180)+cos(ori_A * pi/180)*sin(ori_O * pi/180)*cos(ori_T * pi/180)...
%                 cos(ori_O * pi/180)*cos(ori_T * pi/180)-cos(ori_A * pi/180)*sin(ori_O * pi/180)*sin(ori_T * pi/180)...
%                 sin(ori_A * pi/180)*sin(ori_O * pi/180);...
%                 
%                 -sin(ori_A * pi/180)*cos(ori_T * pi/180)...
%                 sin(ori_A * pi/180)*sin(ori_T * pi/180)...
%                 cos(ori_A * pi/180)...
%                 ]
            
%direct orientation task
% ori_A=0;
% ori_T=0;
% ori_O =0;
% nxp=cos(ori_A * pi/180)*cos(ori_O * pi/180)*cos(ori_T * pi/180)-sin(ori_O * pi/180)*sin(ori_T * pi/180);
% sxp=-sin(ori_O * pi/180)*cos(ori_T * pi/180)-cos(ori_A * pi/180)*cos(ori_O * pi/180)*sin(ori_T * pi/180); 
% axp=sin(ori_A * pi/180)*cos(ori_O * pi/180); 
% 
% nyp= cos(ori_O * pi/180)*sin(ori_T * pi/180)+cos(ori_A * pi/180)*sin(ori_O * pi/180)*cos(ori_T * pi/180); 
% syp=cos(ori_O * pi/180)*cos(ori_T * pi/180)-cos(ori_A * pi/180)*sin(ori_O * pi/180)*sin(ori_T * pi/180); 
% ayp=sin(ori_A * pi/180)*sin(ori_O * pi/180); 
% 
% nzp=-sin(ori_A * pi/180)*cos(ori_T * pi/180); 
% szp= sin(ori_A * pi/180)*sin(ori_T * pi/180); 
% azp=cos(ori_A * pi/180); 

%Preparation to inverse kinematic solution
% nx = ((cos(Q(2)+Q(3))*cos(Q(1))*cos(Q(4))-sin(Q(1))*sin(Q(4)))*cos(Q(5))+(-sin(Q(2)+Q(3))*cos(Q(1)))*sin(Q(5)))*cos(Q(6))+((-cos(Q(4))*sin(Q(1))-cos(Q(2)+Q(3))*cos(Q(1))*sin(Q(4)))*sin(Q(6)));
% ny = ((cos(Q(1))*sin(Q(4))+cos(Q(2)+Q(3))*cos(Q(4))*sin(Q(1)))*cos(Q(5))+(-sin(Q(2)+Q(3))*sin(Q(1)))*sin(Q(5)))*cos(Q(6))+((cos(Q(1))*cos(Q(4))-cos(Q(2)+Q(3))*sin(Q(1))*sin(Q(4)))*sin(Q(6)));
% nz = ((-sin(Q(2)+Q(3))*cos(Q(4)))*cos(Q(5))+(-cos(Q(2)+Q(3)))*sin(Q(5)) )*cos(Q(6))+(sin(Q(2)+Q(3))*sin(Q(4)))*sin(Q(6));
% 
% sx = (-cos(Q(4))*sin(Q(1))-cos(Q(2)+Q(3))*cos(Q(1))*sin(Q(4)))*cos(Q(6))-((cos(Q(2)+Q(3))*cos(Q(1))*cos(Q(4))-sin(Q(1))*sin(Q(4)))*cos(Q(5))+(-sin(Q(2)+Q(3))*cos(Q(1)))*sin(Q(5)) )*sin(Q(6))
% sy = (cos(Q(1))*cos(Q(4))-cos(Q(2)+Q(3)) *sin(Q(1))*sin(Q(4)))*cos(Q(6))-((cos(Q(1))*sin(Q(4))*cos(Q(4))+cos(Q(2)+Q(3))*cos(Q(4)))*cos(Q(5))+(-sin(Q(2)+Q(3))*sin(Q(1)))*sin(Q(5)) )*sin(Q(6))
% sz = (sin(Q(2)+Q(3))*sin(Q(4)) )*cos(Q(6)) -((-sin(Q(2)+Q(3))*cos(Q(4))) *cos(Q(5)) +(-cos(Q(2)+Q(3)) ))*sin(Q(5))*sin(Q(6));
% 
% ax = ((cos(Q(2)+Q(3))*cos(Q(1))*cos(Q(4))-sin(Q(1))*sin(Q(4)))*sin(Q(5))-(-sin(Q(2)+Q(3))*cos(Q(1)))*cos(Q(5))  );
% ay = ((cos(Q(1))*sin(Q(4))+cos(Q(2)+Q(3))*cos(Q(4))*sin(Q(1)))*sin(Q(5))-(-sin(Q(2)+Q(3))*sin(Q(1)))*cos(Q(5))  );
% az = (((-sin(Q(2)+Q(3))*cos(Q(4)))*sin(Q(5)))-(-cos(Q(2)+Q(3)))*cos(Q(5)) );
%  
% px = (cos(Q(1))*cos(Q(2))*a(2)-sin(Q(1))*d(2)+cos(Q(2)+Q(3))*cos(Q(1))*a(3)+sin(Q(2)+Q(3))*cos(Q(1))*d(4))+((cos(Q(2)+Q(3))*cos(Q(1))*cos(Q(4))-sin(Q(1))*sin(Q(4)))*sin(Q(5))-(-sin(Q(2)+Q(3))*cos(Q(1)))*cos(Q(5)))*d(6);
% py = (cos(Q(1))*d(2)+cos(Q(2))*sin(Q(1))*a(2)+cos(Q(2)+Q(3))*sin(Q(1))*a(3)+sin(Q(2)+Q(3))*sin(Q(1))*d(4))+((cos(Q(1))*sin(Q(4))+cos(Q(2)+Q(3))*cos(Q(4))*sin(Q(1)))*sin(Q(5))-(-sin(Q(2)+Q(3))*sin(Q(1)))*cos(Q(5)))*d(6);
% %todo
% pz = (cos(Q(2)+Q(3))*d(4)-sin(Q(2)+Q(3))*a(3)-sin(Q(2))*(a(2)+d(1)))+(((-sin(Q(2)+Q(3))*cos(Q(4)))*sin(Q(5)))-(-cos(Q(2)+Q(3)))*cos(Q(5)) )*d(6);

%temp_A = [nx sx ax px; ny sy ay py; nz sz az pz ]

p4x = px - d(6)*ax;
p4y = py - d(6)*ay;
p4z = pz - d(6)*az;
%p4 = [p4x; p4y; p4z];

% Inverse algebraic kinematic solution

% 1
Q1 = atan22(-sign_hand*p4x*sqrt(p4x^2+p4y^2-(d(2))^2)+p4y*d(2), -sign_hand*p4y*sqrt(p4x^2+p4y^2-(d(2))^2)-p4x*d(2) );

% 2
sin_a_Q2 = -p4z/sqrt(p4x^2+p4y^2-(d(2))^2+p4z^2);
cos_a_Q2 = -sign_hand*sqrt(p4x^2+p4y^2-(d(2))^2)/sqrt(p4x^2+p4y^2-(d(2))^2+p4z^2);
cos_b_Q2 = (a(2)^2+p4x^2++p4y^2-d(2)^2+p4z^2 - (a(3)^2+d(4)^2))/(2*a(2)*sqrt(p4x^2++p4y^2-d(2)^2+p4z^2));
sin_b_Q2 = sqrt(1-cos_b_Q2^2);

sin_Q2 = sin_a_Q2*cos_b_Q2 +(sign_elbow*sign_hand)*cos_a_Q2*sin_b_Q2 ;
cos_Q2 = cos_a_Q2*cos_b_Q2 -(sign_elbow*sign_hand)*sin_a_Q2*sin_b_Q2 ;
Q2 = atan22(cos_Q2,sin_Q2);

% Q3
sin_b_Q3 = abs(a(3))/sqrt(a(3)^2+d(4)^2);
cos_b_Q3 = d(4)/sqrt(a(3)^2+d(4)^2);
cos_fi_Q3= (a(2)^2+a(3)^2+d(4)^2-(p4x^2++p4y^2-d(2)^2+p4z^2))/(2*a(2)*sqrt(a(3)^2+d(4)^2 ));
sin_fi_Q3= sign_elbow*sign_hand*sqrt(1-cos_fi_Q3^2);

sin_Q3 = -sin_fi_Q3*cos_b_Q3-cos_fi_Q3*sin_b_Q3 ;
cos_Q3 = -cos_fi_Q3*cos_b_Q3+sin_fi_Q3*sin_b_Q3 ;
Q3 = atan22(cos_Q3,sin_Q3);

Q=[Q1; Q2; Q3+pi/2 ];
% 4
sin_Q4 = or_ind * (-sin(Q(1))*ax+cos(Q(1))*ay);
cos_Q4 = or_ind * (cos(Q(1))*cos(Q(2)+Q(3))*ax + sin(Q(1))*cos(Q(2)+Q(3))*ay-sin(Q(2)+Q(3))*az );
Q4 = atan22(cos_Q4,sin_Q4);
Q(4)=Q4;
% 5
sin_Q5 = (cos(Q(1))*cos(Q(4))*cos(Q(2)+Q(3))-sin(Q(1))*sin(Q(4)))*ax +(cos(Q(1))*sin(Q(4))+cos(Q(4))*sin(Q(1))*cos(Q(2)+Q(3)))*ay-cos(Q(4))*sin(Q(2)+Q(3))*az ;
cos_Q5 = (cos(Q(1))*sin(Q(2)+Q(3))*ax + sin(Q(1))*sin(Q(2)+Q(3))*ay)+ cos(Q(2)+Q(3))*az;
Q5 = atan22(cos_Q5, sin_Q5);
Q(5)=Q5;
% 6
sin_Q6 = ((-cos(Q(1))*sin(Q(4))*cos(Q(2)+Q(3))-sin(Q(1))*cos(Q(4)))*nx +(cos(Q(1))*cos(Q(4))- sin(Q(4))*sin(Q(1))*cos(Q(2)+Q(3)))*ny)+sin(Q(4))*sin(Q(2)+Q(3))*nz ;
cos_Q6 = ((cos(Q(2)+Q(3))*cos(Q(1))*cos(Q(4))-sin(Q(1))*sin(Q(4)))*cos(Q(5))+(-sin(Q(2)+Q(3))*cos(Q(1)))*sin(Q(5)))*nx+(((cos(Q(1))*sin(Q(4))+cos(Q(2)+Q(3))*cos(Q(4))*sin(Q(1)))*cos(Q(5))+(-sin(Q(2)+Q(3))*sin(Q(1)))*sin(Q(5))))*ny+ ((-sin(Q(2)+Q(3))*cos(Q(4)))*cos(Q(5))+(-cos(Q(2)+Q(3)))*sin(Q(5)))*nz;  
Q6 = atan22(cos_Q6,sin_Q6);


end

%ОЗК с расчётом флагов
function  [Q_out]= inverceKinematic(x,y,z,A) 
R=[0;0;0;1];
%d=[0;150;0;430;0;60];
%a=[0;430;-20;0;0;0];
alfa=[-pi/2; 0; pi/2;-pi/2;pi/2;0];

H = [A [x; y; z]];
H(4,:) = [0 0 0 1];

R=H*R;
%Q=Q*180/pi;
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
                ]
            
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
nx = ((cos(Q(2)+Q(3))*cos(Q(1))*cos(Q(4))-sin(Q(1))*sin(Q(4)))*cos(Q(5))+(-sin(Q(2)+Q(3))*cos(Q(1)))*sin(Q(5)))*cos(Q(6))+((-cos(Q(4))*sin(Q(1))-cos(Q(2)+Q(3))*cos(Q(1))*sin(Q(4)))*sin(Q(6)));
ny = ((cos(Q(1))*sin(Q(4))+cos(Q(2)+Q(3))*cos(Q(4))*sin(Q(1)))*cos(Q(5))+(-sin(Q(2)+Q(3))*sin(Q(1)))*sin(Q(5)))*cos(Q(6))+((cos(Q(1))*cos(Q(4))-cos(Q(2)+Q(3))*sin(Q(1))*sin(Q(4)))*sin(Q(6)));
nz = ((-sin(Q(2)+Q(3))*cos(Q(4)))*cos(Q(5))+(-cos(Q(2)+Q(3)))*sin(Q(5)) )*cos(Q(6))+(sin(Q(2)+Q(3))*sin(Q(4)))*sin(Q(6));

sx = (-cos(Q(4))*sin(Q(1))-cos(Q(2)+Q(3))*cos(Q(1))*sin(Q(4)))*cos(Q(6))-((cos(Q(2)+Q(3))*cos(Q(1))*cos(Q(4))-sin(Q(1))*sin(Q(4)))*cos(Q(5))+(-sin(Q(2)+Q(3))*cos(Q(1)))*sin(Q(5)) )*sin(Q(6));
sy = (cos(Q(1))*cos(Q(4))-cos(Q(2)+Q(3)) *sin(Q(1))*sin(Q(4)))*cos(Q(6))-((cos(Q(1))*sin(Q(4))*cos(Q(4))+cos(Q(2)+Q(3))*cos(Q(4)))*cos(Q(5))+(-sin(Q(2)+Q(3))*sin(Q(1)))*sin(Q(5)) )*sin(Q(6));
sz = (sin(Q(2)+Q(3))*sin(Q(4)) )*cos(Q(6)) -((-sin(Q(2)+Q(3))*cos(Q(4))) *cos(Q(5)) +(-cos(Q(2)+Q(3)) ))*sin(Q(5))*sin(Q(6));

ax = ((cos(Q(2)+Q(3))*cos(Q(1))*cos(Q(4))-sin(Q(1))*sin(Q(4)))*sin(Q(5))-(-sin(Q(2)+Q(3))*cos(Q(1)))*cos(Q(5))  );
ay = ((cos(Q(1))*sin(Q(4))+cos(Q(2)+Q(3))*cos(Q(4))*sin(Q(1)))*sin(Q(5))-(-sin(Q(2)+Q(3))*sin(Q(1)))*cos(Q(5))  );
az = (((-sin(Q(2)+Q(3))*cos(Q(4)))*sin(Q(5)))-(-cos(Q(2)+Q(3)))*cos(Q(5)) );
 
px = (cos(Q(1))*cos(Q(2))*a(2)-sin(Q(1))*d(2)+cos(Q(2)+Q(3))*cos(Q(1))*a(3)+sin(Q(2)+Q(3))*cos(Q(1))*d(4))+((cos(Q(2)+Q(3))*cos(Q(1))*cos(Q(4))-sin(Q(1))*sin(Q(4)))*sin(Q(5))-(-sin(Q(2)+Q(3))*cos(Q(1)))*cos(Q(5)))*d(6);
py = (cos(Q(1))*d(2)+cos(Q(2))*sin(Q(1))*a(2)+cos(Q(2)+Q(3))*sin(Q(1))*a(3)+sin(Q(2)+Q(3))*sin(Q(1))*d(4))+((cos(Q(1))*sin(Q(4))+cos(Q(2)+Q(3))*cos(Q(4))*sin(Q(1)))*sin(Q(5))-(-sin(Q(2)+Q(3))*sin(Q(1)))*cos(Q(5)))*d(6);
%todo
pz = (cos(Q(2)+Q(3))*d(4)-sin(Q(2)+Q(3))*a(3)-sin(Q(2))*(a(2)+d(1)))+(((-sin(Q(2)+Q(3))*cos(Q(4)))*sin(Q(5)))-(-cos(Q(2)+Q(3)))*cos(Q(5)) )*d(6);

temp_A = [nx sx ax px; ny sy ay py; nz sz az pz ];

p4x = px - d(6)*ax;
p4y = py - d(6)*ay;
p4z = pz - d(6)*az;
p4 = [p4x; p4y; p4z];

% Inverse algebraic kinematic solution
% indicator hand calculation:
sign_hand = 1;
sign_hand_t = -d(4)*sin(Q(2)+Q(3))-a(3)*cos(Q(2)+Q(3))-a(2)*cos(Q(2));
if(sign_hand_t<0) sign_hand = -1; end  

% 1
Q1 = atan22(-sign_hand*p4x*sqrt(p4x^2+p4y^2-(d(2))^2)+p4y*d(2), -sign_hand*p4y*sqrt(p4x^2+p4y^2-(d(2))^2)-p4x*d(2) )*180/pi;
% indicator elbow calculation:
sign_elbow = 1;
sign_elbow_t =  sign_hand*(d(4)*cos(Q(3))-a(3)*sin(Q(3)) ) ;
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
Z3 = [ cos(Q(1))*sin(Q(2)+Q(3)); sin(Q(1))*sin(Q(2)+Q(3)); cos(Q(2)+Q(3))];
vector_a = [ax;ay;az];
z3vector_a = cross(Z3,vector_a); 
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
Q=[Q1; Q2; Q3+pi/2 ];
% 4
sin_Q4 = or_ind * (-sin(Q(1))*ax+cos(Q(1))*ay);
cos_Q4 = or_ind * (cos(Q(1))*cos(Q(2)+Q(3))*ax + sin(Q(1))*cos(Q(2)+Q(3))*ay-sin(Q(2)+Q(3))*az );
Q4 = atan22(cos_Q4,sin_Q4);
Q(4)=Q4;
% 5
sin_Q5 = (cos(Q(1))*cos(Q(4))*cos(Q(2)+Q(3))-sin(Q(1))*sin(Q(4)))*ax +(cos(Q(1))*sin(Q(4))+cos(Q(4))*sin(Q(1))*cos(Q(2)+Q(3)))*ay-cos(Q(4))*sin(Q(2)+Q(3))*az ;
cos_Q5 = (cos(Q(1))*sin(Q(2)+Q(3))*ax + sin(Q(1))*sin(Q(2)+Q(3))*ay)+ cos(Q(2)+Q(3))*az;
Q5 = atan22(cos_Q5, sin_Q5);
Q(5)=Q5;
% 6
sin_Q6 = ((-cos(Q(1))*sin(Q(4))*cos(Q(2)+Q(3))-sin(Q(1))*cos(Q(4)))*nx +(cos(Q(1))*cos(Q(4))- sin(Q(4))*sin(Q(1))*cos(Q(2)+Q(3)))*ny)+sin(Q(4))*sin(Q(2)+Q(3))*nz ;
cos_Q6 = ((cos(Q(2)+Q(3))*cos(Q(1))*cos(Q(4))-sin(Q(1))*sin(Q(4)))*cos(Q(5))+(-sin(Q(2)+Q(3))*cos(Q(1)))*sin(Q(5)))*nx+(((cos(Q(1))*sin(Q(4))+cos(Q(2)+Q(3))*cos(Q(4))*sin(Q(1)))*cos(Q(5))+(-sin(Q(2)+Q(3))*sin(Q(1)))*sin(Q(5))))*ny+ ((-sin(Q(2)+Q(3))*cos(Q(4)))*cos(Q(5))+(-cos(Q(2)+Q(3)))*sin(Q(5)))*nz;  
Q6 = atan22(cos_Q6,sin_Q6);

Q_out = [Q1; Q2; Q3; Q4; Q5; Q6];
end

%рассчёт флагов
function [sign_hand, sign_elbow, or_ind]= CalculateI(Q)
R=[0;0;0;1];
d=[0;150;0;430;0;60];
a=[0;430;-20;0;0;0];
alfa=[-pi/2; 0; pi/2;-pi/2;pi/2;0];
% A1=getA(1,Q);
% A2=getA(2,Q);
% A3=getA(3,Q);
% A4=getA(4,Q);
% A5=getA(5,Q);
% A6=getA(6,Q);

H=getA2(1,6,Q);
R=H*R;
%Q=Q*180/pi;
nx= H( 1,1 ); sx= H(1 ,2 ); ax= H( 1, 3); px= H(1 , 4);
ny= H( 2,1 ); sy= H(2 ,2 ); ay= H( 2, 3); py= H(2 , 4);
nz= H( 3,1 ); sz= H(3 ,2 ); az= H( 3, 3); pz= H(3 , 4);

%inverse orientation task
% ori_T = atan22(nz,-sz)*180/pi ;
% ori_A = atan22(az, -nz*cos(ori_T*pi/180)+sz*sin(ori_T*pi/180))*180/pi;
% ori_O = atan22(ny*sin(ori_T*pi/180)+sy*cos(ori_T*pi/180), -nx*sin(ori_T*pi/180)-sx*cos(ori_T*pi/180) )*180/pi; 

% ori_Matrix = [  cos(ori_A * pi/180)*cos(ori_O * pi/180)*cos(ori_T * pi/180)-sin(ori_O * pi/180)*sin(ori_T * pi/180)...
%                 -sin(ori_O * pi/180)*cos(ori_T * pi/180)-cos(ori_A * pi/180)*cos(ori_O * pi/180)*sin(ori_T * pi/180)...
%                 sin(ori_A * pi/180)*cos(ori_O * pi/180);...
%                 
%                 cos(ori_O * pi/180)*sin(ori_T * pi/180)+cos(ori_A * pi/180)*sin(ori_O * pi/180)*cos(ori_T * pi/180)...
%                 cos(ori_O * pi/180)*cos(ori_T * pi/180)-cos(ori_A * pi/180)*sin(ori_O * pi/180)*sin(ori_T * pi/180)...
%                 sin(ori_A * pi/180)*sin(ori_O * pi/180);...
%                 
%                 -sin(ori_A * pi/180)*cos(ori_T * pi/180)...
%                 sin(ori_A * pi/180)*sin(ori_T * pi/180)...
%                 cos(ori_A * pi/180)...
%                 ]
%             

%Preparation to inverse kinematic solution
% nx = ((cos(Q(2)+Q(3))*cos(Q(1))*cos(Q(4))-sin(Q(1))*sin(Q(4)))*cos(Q(5))+(-sin(Q(2)+Q(3))*cos(Q(1)))*sin(Q(5)))*cos(Q(6))+((-cos(Q(4))*sin(Q(1))-cos(Q(2)+Q(3))*cos(Q(1))*sin(Q(4)))*sin(Q(6)));
% ny = ((cos(Q(1))*sin(Q(4))+cos(Q(2)+Q(3))*cos(Q(4))*sin(Q(1)))*cos(Q(5))+(-sin(Q(2)+Q(3))*sin(Q(1)))*sin(Q(5)))*cos(Q(6))+((cos(Q(1))*cos(Q(4))-cos(Q(2)+Q(3))*sin(Q(1))*sin(Q(4)))*sin(Q(6)));
% nz = ((-sin(Q(2)+Q(3))*cos(Q(4)))*cos(Q(5))+(-cos(Q(2)+Q(3)))*sin(Q(5)) )*cos(Q(6))+(sin(Q(2)+Q(3))*sin(Q(4)))*sin(Q(6));
% 
% sx = (-cos(Q(4))*sin(Q(1))-cos(Q(2)+Q(3))*cos(Q(1))*sin(Q(4)))*cos(Q(6))-((cos(Q(2)+Q(3))*cos(Q(1))*cos(Q(4))-sin(Q(1))*sin(Q(4)))*cos(Q(5))+(-sin(Q(2)+Q(3))*cos(Q(1)))*sin(Q(5)) )*sin(Q(6));
% sy = (cos(Q(1))*cos(Q(4))-cos(Q(2)+Q(3)) *sin(Q(1))*sin(Q(4)))*cos(Q(6))-((cos(Q(1))*sin(Q(4))*cos(Q(4))+cos(Q(2)+Q(3))*cos(Q(4)))*cos(Q(5))+(-sin(Q(2)+Q(3))*sin(Q(1)))*sin(Q(5)) )*sin(Q(6));
% sz = (sin(Q(2)+Q(3))*sin(Q(4)) )*cos(Q(6)) -((-sin(Q(2)+Q(3))*cos(Q(4))) *cos(Q(5)) +(-cos(Q(2)+Q(3)) ))*sin(Q(5))*sin(Q(6));
% 
% ax = ((cos(Q(2)+Q(3))*cos(Q(1))*cos(Q(4))-sin(Q(1))*sin(Q(4)))*sin(Q(5))-(-sin(Q(2)+Q(3))*cos(Q(1)))*cos(Q(5))  );
% ay = ((cos(Q(1))*sin(Q(4))+cos(Q(2)+Q(3))*cos(Q(4))*sin(Q(1)))*sin(Q(5))-(-sin(Q(2)+Q(3))*sin(Q(1)))*cos(Q(5))  );
% az = (((-sin(Q(2)+Q(3))*cos(Q(4)))*sin(Q(5)))-(-cos(Q(2)+Q(3)))*cos(Q(5)) );
%  
% px = (cos(Q(1))*cos(Q(2))*a(2)-sin(Q(1))*d(2)+cos(Q(2)+Q(3))*cos(Q(1))*a(3)+sin(Q(2)+Q(3))*cos(Q(1))*d(4))+((cos(Q(2)+Q(3))*cos(Q(1))*cos(Q(4))-sin(Q(1))*sin(Q(4)))*sin(Q(5))-(-sin(Q(2)+Q(3))*cos(Q(1)))*cos(Q(5)))*d(6);
% py = (cos(Q(1))*d(2)+cos(Q(2))*sin(Q(1))*a(2)+cos(Q(2)+Q(3))*sin(Q(1))*a(3)+sin(Q(2)+Q(3))*sin(Q(1))*d(4))+((cos(Q(1))*sin(Q(4))+cos(Q(2)+Q(3))*cos(Q(4))*sin(Q(1)))*sin(Q(5))-(-sin(Q(2)+Q(3))*sin(Q(1)))*cos(Q(5)))*d(6);
% pz = (cos(Q(2)+Q(3))*d(4)-sin(Q(2)+Q(3))*a(3)-sin(Q(2))*(a(2)+d(1)))+(((-sin(Q(2)+Q(3))*cos(Q(4)))*sin(Q(5)))-(-cos(Q(2)+Q(3)))*cos(Q(5)) )*d(6);
% 
% temp_A = [nx sx ax px; ny sy ay py; nz sz az pz ];

p4x = px - d(6)*ax;
p4y = py - d(6)*ay;
p4z = pz - d(6)*az;
%p4 = [p4x; p4y; p4z];

% Inverse algebraic kinematic solution
% indicator hand calculation:

sign_hand_t = -d(4)*sin(Q(2)+Q(3))-a(3)*cos(Q(2)+Q(3))-a(2)*cos(Q(2));
if(sign_hand_t<0)   sign_hand = -1; 
else                sign_hand = 1;
end

% 1
% Q1 = atan22(-sign_hand*p4x*sqrt(p4x^2+p4y^2-(d(2))^2)+p4y*d(2), -sign_hand*p4y*sqrt(p4x^2+p4y^2-(d(2))^2)-p4x*d(2) )*180/pi;
% indicator elbow calculation:

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

% ПЗК + изобразить модель робота при showFlag=1, 
function  [x,y,z,A]=robot_arm(q,showFlag)

    if(showFlag==1)
        cla 
        [x,y,z,A]= drawRobot(q);
        [a,b,c]=MatrixToEuler(A);
        drawnow
        xlabel('X = '+string(round(x))+' C = '+string(round(c*180/pi)));
        ylabel('Y = '+string(round(y))+' B = '+string(round(b*180/pi)));
        zlabel('Z = '+string(round(z))+' A = '+string(round(a*180/pi)));
        drawnow
    else
        [x,y,z,A]= forwardKinematics(q);
    end
end

% ПЗК и флаги ориентации + изобразить модель робота при showFlag=1,
function  [x,y,z,A,i1,i2,i3]=robot_arm_I(q,showFlag)

    if(showFlag==1)
        cla 
        [x,y,z,A]= drawRobot(q);
        [i1,i2,i3] = CalculateI(q);
        [a,b,c]=MatrixToEuler(A);
        title('The graphic of robot-arm| I1 = '+string(i1)+'; I2 = '+string(i2)+'; I3 = '+string(i3));
        xlabel('X = '+string(round(x))+' C = '+string(round(c*180/pi)));
        ylabel('Y = '+string(round(y))+' B = '+string(round(b*180/pi)));
        zlabel('Z = '+string(round(z))+' A = '+string(round(a*180/pi)));
        drawnow
    else
        [x,y,z,A]= forwardKinematics(q);
        [i1,i2,i3] = CalculateI(q);
    end
end

% ОЗК + изобразить модель робота при showFlag=1, reachFlag =0 в слечае
% невозможности достижения точки
function [q_M, reachFlag]= robot_arm_lin(x,y,z,A,i1,i2,i3,showFlag)

    try
        q_M= inverceKinematicI(x,y,z,A, i1, i2, i3);
        reachFlag=1;
        if(showFlag==1)
            cla 
            drawRobot(q_M);
            drawnow
            title('The graphic of robot-arm| I1 = '+string(i1)+'; I2 = '+string(i2)+'; I3 = '+string(i3));
            xlabel('X = '+string(round(x)));
            ylabel('Y = '+string(round(y)));
            zlabel('Z = '+string(round(z)));
        end
    catch
        title('impossible to achieve purpose pozicion');
            q_M= [0;0;0;0;0;0];
            reachFlag=0;
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%возвращает координаты центра масс звеньев
function [r_i]=r(i,q)
 r_0 = [0;0;0;1];
 d=[0;150;0;430;0;60];
 a=[0;430;-20;0;0;0];

 if(i==0)
    r_i = r_0;
 elseif(i==1)
    r_i =  getA(1,q)*[0; -50; 0 ; 1];
 elseif(i==2)
    r_i =  getA2(1,2,q)*[ -a(2)/2; 0; -d(2)/2 ; 1];
 elseif(i==3)
    r_i = getA2(1,3,q)*[- a(3)/2; 0; 0 ; 1];
 elseif(i==4)
    r_i = getA2(1,4,q)*[0;  d(4)/2; 0 ; 1];
 elseif(i==5)
   r_i = getA2(1,5,q)*[0; 0;  d(6)/2 ; 1];
 elseif(i==6)
   r_i = getA2(1,6,q)*[0; 0; - 10 ; 1];          
 end
 
end

%отобразить координаты центров масс
function [r_i]=r_draw(i,q) 
r_i = r(i,q);
 
pr1=drawPoint(r_i);
pr1.Marker='h';
pr1.LineWidth=1;
text(r_i(1),r_i(2),r_i(3),'m'+string(i));
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
%перевод из самолётных углов в матрицу поворота, TO-DO
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

% перевод матрицы поворота в самолётные углы, TO-DO
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

% настройка вывода граффика
function  begin()
    cla
    %axes('Xlim',[-900 900], 'Ylim',[-900 900], 'Zlim',[-500 900]);
    view(3)
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

function drawMatrix(A,size,str) 

p= A(:,4).';
% AM=[size 0 0 0; 0 size 0 0; 0 0 size  0; 0 0 0 size];
px=A(:,1).';
py=A(:,2).';
pz=A(:,3).';
px(4)=[];
py(4)=[];
pz(4)=[];
p(4)=[];

px= transferPoint(px*size,p);
py= transferPoint(py*size,p);
pz= transferPoint(pz*size,p);

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

%построение матрицы по параметрам Денавита–Хартенберга
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

% тот же atan2, только меняет местами аргументы
function fi = atan22(x,y) 
 fi=atan2(y,x);
end

 