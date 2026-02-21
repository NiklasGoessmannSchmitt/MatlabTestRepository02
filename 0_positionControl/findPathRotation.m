function [m] = findPathRotation(Px, Py, t)

dx = 2*(1-t)*(Px(2)-Px(1))+2*t*(Px(3)-Px(2));
dy = 2*(1-t)*(Py(2)-Py(1))+2*t*(Py(3)-Py(2));
if dy < 0 
    m = -acos(dx/sqrt(dx^2+dy^2));
else
    m = acos(dx/sqrt(dx^2+dy^2));
end
    
% 
% granularity = 10e-6;
% iterator = round(iterator*1/granularity)*granularity;
% xOld = 0; 
% yOld = 0; 
% tOld = 0; 
% for t = 0:granularity:iterator
%     x = (1-t)^2*Px(1) + 2*(1-t)*t*Px(2) + t^2*Px(3);
%     y = (1-t)^2*Py(1) + 2*(1-t)*t*Py(2) + t^2*Py(3);      
%     
%     theta = atan2( (y-yOld), (x-xOld) );
%     alpha = theta - tOld;
%         
%     xOld = x; 
%     yOld = y; 
%     tOld = theta;
% end

end