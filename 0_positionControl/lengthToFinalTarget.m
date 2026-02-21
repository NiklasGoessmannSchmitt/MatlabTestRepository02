function arclength = lengthToFinalTarget(xID, yID, dq, AGVindex)
% adds the legnth of the path to up until the next Identpoint along dq
[a,b] = size(xID);
[c,d] = size(yID);

if a == 1 && b == 1 && c == 1 && d == 1 
    
    arclength = sum(dq(AGVindex:xID-1));

end

    
