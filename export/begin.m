% настройка вывода граффика
function  begin()
    cla
    %axes('Xlim',[-900 900], 'Ylim',[-900 900], 'Zlim',[-500 900]);
    view(3)
    axis equal; grid on; hold on; 
    drawCoordSystem(5);
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