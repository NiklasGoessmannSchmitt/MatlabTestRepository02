function [indexx, angle] = findNearestPoint(pXs, pYs, pX, pY, pA, x, y)

pAmod = round(pA*180/pi); 
pAmin = pAmod - 20;
pAmax = pAmod + 20; 

% angleIterator = pAmin:0.1:pAmax;
% angleIterator = -20:0.1:20;
% pXsRot = pX + sqrt((pXs-pX)^2+(pYs-pY)^2) * cos(angleIterator/180*pi); 
% pYsRot = pY + sqrt((pXs-pX)^2+(pYs-pY)^2) * sin(angleIterator/180*pi); 
% plot(pXsRot,pYsRot,'m')
% plot(pX + sqrt((pXs-pX)^2+(pYs-pY)^2) * cos(angleIterator(1)/180*pi), pY + sqrt((pXs-pX)^2+(pYs-pY)^2) * sin(angleIterator(1)/180*pi), 'bo')
% plot(pX + sqrt((pXs-pX)^2+(pYs-pY)^2) * cos(angleIterator(end)/180*pi), pY + sqrt((pXs-pX)^2+(pYs-pY)^2) * sin(angleIterator(end)/180*pi), 'ro')

for iterator = 1:length(x)   
    for angleIterator = pAmin:0.01:pAmax
        pXsRot = pX + sqrt((pXs-pX)^2+(pYs-pY)^2) * cos(angleIterator/180*pi); 
        pYsRot = pY + sqrt((pXs-pX)^2+(pYs-pY)^2) * sin(angleIterator/180*pi); 
        if iterator == 1 && angleIterator == pAmin
            shortestDistance = sqrt( (pXsRot-x(iterator))^2 + (pYsRot-y(iterator))^2 ); 
            indexx = iterator;
            angle = angleIterator;
        elseif shortestDistance > sqrt( (pXsRot-x(iterator))^2 + (pYsRot-y(iterator))^2 )
            shortestDistance = sqrt( (pXsRot-x(iterator))^2 + (pYsRot-y(iterator))^2 );
            indexx = iterator;
            angle = angleIterator;
        end      
    end
end

end