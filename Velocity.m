function Velocity

q0=[0; 1; 2; 0; 5; 0];
q=q0+1
q2=q0-1
vQ= deriv(q0)
vQ2= deriv(q0+2)
tk=100
step = 10/tk;
t = 1:1:tk;

   for k =1:tk
      vQ=vQ.derStep(q,step);
      vQ2=vQ2.derStep(q2,step);
      v(k)=vQ.dX(1)
      a(k)=vQ.d2X(1)
      vQ2.dX'
      q=q+k/10;
      q2=q2-1/10;
   end
plot(t,v,t,a)

end


