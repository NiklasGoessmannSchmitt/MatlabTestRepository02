function [Array] = moveCoordinateSystem(Points,angle,originX,originY)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%Translation
for n=1:length(Points)
Array_translation(n).x=Points(n).x - (originX);
Array_translation(n).y=Points(n).y - (originY);
end

%Rotation
for k=1:length(Points)
Array(k).x=Array_translation(k).x*cos(angle) + Array_translation(k).y*sin(angle);
Array(k).y=-Array_translation(k).x*sin(angle)+ Array_translation(k).y*cos(angle);  
end




end