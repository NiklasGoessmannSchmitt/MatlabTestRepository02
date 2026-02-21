function [x,y] = bezier3(Px,Py,n)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

granularity = 1/n;
x = zeros(1,n); 
y = zeros(1,n); 
b = 1;

if length(Px)==3
    for t = 0:granularity:1
        x(b) = (1-t)^2*Px(1) + 2*(1-t)*t*Px(2) + t^2*Px(3);
        y(b) = (1-t)^2*Py(1) + 2*(1-t)*t*Py(2) + t^2*Py(3);
        b = b + 1;
    end

elseif length(Px)==4
    for t = 0:granularity:1
        x(b) = (1-t)^3*Px(1)+ 3*t*(1-t)^2*Px(2) + 3*t^2*(1-t)*Px(3) + t^3*Px(4) ;
        y(b) = (1-t)^3*Py(1)+ 3*t*(1-t)^2*Py(2) + 3*t^2*(1-t)*Py(3) + t^3*Py(4);
        b = b + 1;
    end

end

end

