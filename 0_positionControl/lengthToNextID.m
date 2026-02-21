function arcLength = lengthToNextID(xID, yID, xq,yq, dq, AGVindex, xy)
% arcLength = lengthToNextID(xID, yID, xq,yq,AGVindex, xy)
%
% in the current implementation this section can only integrate in one
% direction 

indx = AGVindex; 

[a,b] = size(xID);
[c,d] = size(yID);

if a == 1 && b == 1 && c == 1 && d == 1 
    
    for numbersOfSamplesPushed = 1:9999999

        % when endpoint and startpoint match --> close the loop [xq(n),yq(n)]->[xq(0),yq(0)]
        if (indx+1+numbersOfSamplesPushed) > length(xq) && ...
                (xq(1) - xq(end)) > 0.01 && ...
                (yq(1) - yq(end)) > 0.01
            indx = indx - length(xq) + 1;
        end

        if numbersOfSamplesPushed == 1  
            % first integration from closest point xy to [xq(a),yq(a)]
            arcLength = sqrt( (xq(indx+1)-xy(1))^2 + (yq(indx+1)-xy(2))^2 );
        else 
            if (indx+1+numbersOfSamplesPushed) < length(xq)
                % continue integration from [xq(a),yq(a)]->[xq(a+1),yq(a+1)]
                arcLength = arcLength + dq(indx+numbersOfSamplesPushed);
               % arcLength = arcLength + sqrt( (xq(indx+1+numbersOfSamplesPushed) - xq(indx+1+(numbersOfSamplesPushed-1)))^2 + (yq(indx+1+numbersOfSamplesPushed) - yq(indx+1+(numbersOfSamplesPushed-1)))^2);
            end
        end
        if (indx+1+numbersOfSamplesPushed) >= xID && (indx+1+numbersOfSamplesPushed) >= yID
            break;
        end  
    end
end