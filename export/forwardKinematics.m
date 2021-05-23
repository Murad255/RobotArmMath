%ПЗК
function [x,y,z,A]=forwardKinematics(A1,A2,A3,A4,A5,A6)
q=[A1,A2,A3,A4,A5,A6];
 q(3) = q(3)+pi/2;
Tk=getA2(1,6,q);
A=Tk;
A(:,4)=[]; A(4,:)=[];
R=[0;0;0;1];
t=Tk*R;
x=t(1); y=t(2); z=t(3);
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

 