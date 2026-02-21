function [outputArg1,outputArg2] = GraphsDeviation(x_k,y_k, P)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here



if length(P.x)==3

    for i= 1:length(x_k)
        %dist is a function of the pythagoras between current position and the bezier segments:
         distf{i}=@(t) sqrt((((1-t).^2*P.x(1) + 2*(1-t).*t*P.x(2) + t.^2*P.x(3))-x_k(i))^2 + (((1-t).^2*P.y(1) + 2*(1-t).*t*P.y(2) + t.^2*P.y(3)) - y_k(i))^2);
        [t(1),dista(1)]=fminbnd(distf{i},0,1);
        distg(i)=dista(1);
%         t_s(i)=t(1);
    end

elseif length(P.x)==4
    for i= 1:length(x_k)
        distf{i}=@(t) sqrt((((1-t).^3*P.x(1) + 3*t*(1-t)^2*P.x(2) + 3*t^2*(1-t)*P.x(3) + t^3*P.x(4))-x_k(i))^2 + (((1-t).^3*P.y(1) + 3*t*(1-t)^2*P.y(2) + 3*t^2*(1-t)*P.y(3) + t^3*P.y(4)) - y_k(i))^2);
        [t(1),dista(1)]=fminbnd(distf{i},0,1);
        distg(i)=dista(1);

    end

end


%if curvesegments == 1
%look for minimum in bezier functions between t=0 and t=1

q = 1;
%else
figure 

plot(distg);
xlabel('Iterationsschritte');
ylabel('Abweichung in m^2');
Fehler= trapz(distg)


%end