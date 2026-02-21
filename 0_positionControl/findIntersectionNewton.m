function [x, y, iteratingSteps, shortestDistance] = findIntersectionNewton(pXs, pYs, pXe, pYe, Px_bezier, Py_bezier)

t = 0.5;
stepSize = 0.25;
iteratingSteps = 0; 
shortestDistance = 1;
failureTolerance = 1e-9;

plot([pXe pXs], [pYe pYs])

while (abs(shortestDistance) > failureTolerance) && (iteratingSteps < 100)
    
    x_bezier = (1-t)^2*Px_bezier(1) + 2*(1-t)*t*Px_bezier(2) + t^2*Px_bezier(3);
    y_bezier = (1-t)^2*Py_bezier(1) + 2*(1-t)*t*Py_bezier(2) + t^2*Py_bezier(3);
    
    plot(x_bezier, y_bezier, 'r+')
    
    smallestIterationError = 1;
    smallestIterationIterator = 0;
    for linearIterator = 0:0.001:1
        
        x_linear = pXs + (pXe - pXs) * linearIterator;
        y_linear = pYs + (pYe - pYs) * linearIterator;
        
%         if iteratingSteps == 1 
%             plot(x_linear,y_linear,'r.')
%         end
        % fehlerberechnung
        tempError = sqrt( (x_bezier - x_linear)^2 + (y_bezier - y_linear)^2 );
        
        if abs(tempError) < smallestIterationError 
        	smallestIterationError = abs(tempError); 
            smallestIterationIterator = linearIterator;
        end
            
        if (linearIterator == 0 && iteratingSteps == 0) || abs(shortestDistance) > tempError
            shortestDistance = tempError;
            x = x_linear; 
            y = y_linear;
        end          
    end
    plot(pXs + (pXe - pXs) * smallestIterationIterator,...
         pYs + (pYe - pYs) * smallestIterationIterator,'b+')
    
    if smallestIterationError <= shortestDistance || ...% to do
        x_bezier > x_linear
        t = t - stepSize;
    else
        t = t + stepSize;
    end
    
    iteratingSteps = iteratingSteps + 1;
    stepSize = stepSize / 2;
end

iterator = t; 

end