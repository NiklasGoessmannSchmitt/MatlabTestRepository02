function [deviation, sensorFrameRot, xi, yi] = trackSensor(trackX, trackY, curMapPos, sensor)

persistent lastIntersectionIndx

if isempty(lastIntersectionIndx)
    lastIntersectionIndx = 1;
end

%                              X                Y
sensorFrame = [curMapPos(1)+sensor.pos(1), curMapPos(2);                ... % sensor center
               curMapPos(1)+sensor.pos(1), curMapPos(2)-sensor.range/2; ... % sensor left
               curMapPos(1)+sensor.pos(1), curMapPos(2)+sensor.range/2]';   % sensor right  

% rotate coordinate system  
center = [curMapPos(1); curMapPos(2)];
R = [cos(curMapPos(3)) -sin(curMapPos(3)); sin(curMapPos(3)) cos(curMapPos(3))];
sensorFrameRot = (R*(sensorFrame - center) + center)';

if false 
    figure
    plot(trackX(lastIntersectionIndx:end),trackY(lastIntersectionIndx:end)); hold on 
    plot(sensorFrameRot(2:3,1),sensorFrameRot(2:3,2))
end 

[xi, yi] = intersections(trackX(lastIntersectionIndx:end),trackY(lastIntersectionIndx:end),...
                         sensorFrameRot(2:3,1),sensorFrameRot(2:3,2));

if ~isempty(xi)
    %% Y component
    deviationRight = sqrt( (sensorFrameRot(3,1)-xi(1))^2 + (sensorFrameRot(3,2)-yi(1))^2 );
    deviationLeft  = sqrt( (sensorFrameRot(2,1)-xi(1))^2 + (sensorFrameRot(2,2)-yi(1))^2 );
    if deviationRight < deviationLeft 
        deviation.y = -sqrt( (sensorFrameRot(1,1)-xi(1))^2 + (sensorFrameRot(1,2)-yi(1))^2 );
    else
        deviation.y = sqrt( (sensorFrameRot(1,1)-xi(1))^2 + (sensorFrameRot(1,2)-yi(1))^2 );
    end

    %% A component
    distance = ones(1,length(trackX)-1);
    for b = lastIntersectionIndx:length(trackX)-1 
        distance(b) = sqrt( ((xi(1)-trackX(b))^2 + (yi(1)-trackY(b))^2) );
    end
    [~,indx] = min(distance);  
    lastIntersectionIndx = indx - 10;
    if lastIntersectionIndx < 1
        lastIntersectionIndx = 1; 
    end

    a = [trackX(indx+1) - trackX(indx), trackY(indx+1) - trackY(indx)];
    b = [sensorFrameRot(3,1) - sensorFrameRot(2,1), sensorFrameRot(3,2) - sensorFrameRot(2,2)]; 

    s = dot(a,b);
    t = norm(a)*norm(b); 
    deviation.a = acos(s/t)-pi/2;
else
    deviation.y = 'error';
    deviation.a = 'error';
    disp('error: no intersection between track sensor and track!')
end
end