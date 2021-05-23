classdef deriv
    %  ласс дл€ нахождени€ производной 1-го и 2-го пор€дка
    properties
        X  {mustBeNumeric}
        dX  {mustBeNumeric}
        d2X  {mustBeNumeric}
    end
    
    methods
        function obj = deriv(x0)
            %DERIV Construct an instance of this class
            obj.X = x0;
            s = size(x0);
            obj.dX =zeros(s(1),1);
            obj.d2X =zeros(s(1),1);
        end
        
        function obj = der(obj, newX)
           x0 =obj.X;
           past_dX= obj.dX;
           s = size(newX);
           now_dX=zeros(s(1),1);
           now_d2X=zeros(s(1),1);
           
            if(s(1)>1)
               for k =1:s(1)
                   now_dX(k)=(newX(k)-x0(k));
                   now_d2X(k)=(now_dX(k)-past_dX(k));
               end
            end
            obj.X=newX;
            obj.dX=now_dX;
            obj.d2X=now_d2X;
        end
        
        function obj = derStep(obj, newX, step)
           x0 =obj.X;
           past_dX= obj.dX;
           s = size(newX);
           now_dX=zeros(s(1),1);
           now_d2X=zeros(s(1),1);
           
            if(s(1)>1)
               for k =1:s(1)
                   now_dX(k)=(newX(k)-x0(k))/step;
                   now_d2X(k)=(now_dX(k)-past_dX(k))/step;
               end
            end
            obj.X=newX;
            obj.dX=now_dX;
            obj.d2X=now_d2X;
        end
        
    end
end

