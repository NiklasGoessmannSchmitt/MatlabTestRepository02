function [iterator, angleOverall, shortestOverallDistance] = findNearestPointNewton(pXs, pYs, pX, pY, pA, Px, Py, failureTolerance)

pAmod = round(pA*180/pi); 
pAmin = pAmod - 20;
pAmax = pAmod + 20; 

t = 0.5;
stepSize = 0.25;
iteratingSteps = 0; 
shortestOverallDistance = 1;
% shortestIterationDistance = 1;
% shortestIterationDistanceOld = 3; % to do :: halbe bezier länge
firstLoop = true;
% wasDown = false;
% wasUp = false;
% failureTolerance = 1e-9;
% plot(pX + sqrt((pXs-pX)^2+(pYs-pY)^2) * cos([pAmin:0.001:pAmax]/180*pi), pY + sqrt((pXs-pX)^2+(pYs-pY)^2) * sin([pAmin:0.001:pAmax]/180*pi),'g.')

while (abs(shortestOverallDistance) > failureTolerance) && (iteratingSteps < 100)
    
    x = (1-t)^2*Px(1) + 2*(1-t)*t*Px(2) + t^2*Px(3);
    y = (1-t)^2*Py(1) + 2*(1-t)*t*Py(2) + t^2*Py(3);
    
    shortestIterationDistance = 1; 
    for angleIterator = pAmin:0.001:pAmax
        pXsRot = pX + sqrt((pXs-pX)^2+(pYs-pY)^2) * cos(angleIterator/180*pi); 
        pYsRot = pY + sqrt((pXs-pX)^2+(pYs-pY)^2) * sin(angleIterator/180*pi); 
        if firstLoop || abs(shortestIterationDistance) > sqrt( (pXsRot-x)^2 + (pYsRot-y)^2 )
            shortestIterationDistance = sqrt( (pXsRot-x)^2 + (pYsRot-y)^2 ); 
%             angleIteration = angleIterator/180*pi;
            
            if firstLoop || abs(shortestOverallDistance) > abs(shortestIterationDistance)
                shortestOverallDistance = shortestIterationDistance;
                headingAngle = atan2(y-pYsRot,x-pXsRot); 
                if headingAngle < -pi/2 || headingAngle > pi/2 
                    shortestOverallDistance = -shortestOverallDistance;
                end
                angleOverall = angleIterator/180*pi;                
            end
            
        end      
    end
    
%     plot(x,y,'r+')
    
%     if shortestOverallDistance > 0
%     rightDirection = (shortestIterationDistanceOld > shortestIterationDistance);
%     
%     if rightDirection && (wasDown || firstLoop)
%         down = true; 
%         up = false;
%     elseif rightDirection && (wasUp || firstLoop)
%         up = true;
%         down = false;
%     elseif ~rightDirection && wasUp
%         up = false;
%         down = true;
%     elseif ~rightDirection && wasDown
%         down = false;
%         up = true;
%     else
%         ;
%     end
    
%     d = sqrt( (pX - x)^2 + (pY - y)^2 ); 
%     r = sqrt( (pX-pXs)^2 + (pY-pYs)^2);
%     down = d > r; 
    inside = sqrt( (x-pX)^2 + (y-pY)^2 ) < sqrt( (pX-pXs)^2 + (pY-pYs)^2);
%     headingAngle = atan2(y-pYsRot,x-pXsRot); 
    orientationAim = atan2(pYs-pY,pXs-pX);
    orientationTarget = atan2(y-pY,x-pX);
    overtaken = abs(normAngle(orientationAim-orientationTarget)) > pi/2;
%     overtaken = headingAngle > pi/2; 
    
    down = ~inside & ~overtaken; 
    
    if down %%(shortestIterationDistanceOld > shortestIterationDistance) && (wasDown || firstLoop)  
        t = t - stepSize;
%         wasDown = true;
%         wasUp = false;
    else
        t = t + stepSize;
%         wasDown = false;
%         wasUp = true;
    end
    
%     shortestIterationDistanceOld = shortestIterationDistance;
    iteratingSteps = iteratingSteps + 1;
    stepSize = stepSize / 2;
    firstLoop = false;
end

iterator = t; 

end