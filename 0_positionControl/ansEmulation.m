function [target, finalTargetLength, targetLength, carrotGlobal, xy, indx, c] = ansEmulation(xq, yq,dq, xID, yID, pushLength, g, b) 

%% Find closest point on path
% <<<<<<< HEAD
% xy = distance2curve([xq; yq]',[g.curMapPos(1) g.curMapPos(2)]); 
% xy = distance2bezier3(b,g.curMapPos(1),g.curMapPos(2));
% =======
xy = distance2curve([xq; yq]',[g.curMapPos(1) g.curMapPos(2)]); 
% JONAS xy = distance2bezier3(b,g.curMapPos(1),g.curMapPos(2)); 
%  (up) commented out because it does not work with EWN layout
% >>>>>>> 64ac439722a0774faa9c989dde4c112ca07a7c86
%  xy = distance2beztest(b,g.curMapPos(1),g.curMapPos(2));

% exception missing for finding two valid points

distance = ones(1,length(xq));
for b = 1:length(xq) 
    distance(b) = sqrt( ((xy(1)-xq(b))^2 + (xy(2)-yq(b))^2) );
end
[~,indx] = min(distance); 

%% Integrate distance along the path

[c, targetLength] = pushTargetOnTrack(xq, yq, dq, xID, yID, indx, xy, pushLength);
%[c, targetLength] = pushTargetOnTrack2(xq, yq, dq, xID, yID, indx, xy, pushLength);
finalTargetLength = lengthToNextID(xID(end), yID(end), xq,yq, dq,indx, xy);
%finalTargetLength = lengthToFinalTarget(xID(end),yID(end),dq, indx);


carrotGlobal(1) = xq(indx+c);
carrotGlobal(2) = yq(indx+c);
% carrotGlobal(3) = atan2( ((yq(indx+c+1) - yq(indx+c))), (xq(indx+c+1) - xq(indx+c)) );
carrotGlobal(3) = atan2( ((yq(indx+c) - yq(indx+c-1))), (xq(indx+c) - xq(indx+c-1)) );

vts = carrotGlobal - g.curMapPos;
target = vts; 

%% Transform Target Vector to AGV coordinate system
alpha = atan2(vts(2),vts(1));
target(1) = cos(g.curMapPos(3) - alpha) * sqrt(vts(1)^2+vts(2)^2);
target(2) = cos(g.curMapPos(3)+pi/2 - alpha) * sqrt(vts(1)^2+vts(2)^2);
target(3) = mod(alpha-g.curMapPos(3),2*pi);

if target(3) < -pi

    target(3) = target(3) + 2*pi;

elseif target(3) > pi

    target(3) = target(3) - 2*pi;

end

end
