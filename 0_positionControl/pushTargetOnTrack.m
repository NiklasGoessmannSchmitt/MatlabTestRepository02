function [numbersOfSamplesPushed, arcLength] = pushTargetOnTrack(xq,yq, dq, xIDindx,yIDindx,AGVindex, xy, targetLength)
% [numbersOfSamplesPushed, arcLength] = pushTargetOnTrack(xq,yq,AGVindex)
%
% in the current implementation this section can only integrate in one
% direction 
persistent targetFound 
if isempty(targetFound) 
    targetFound = false; 
end

indx = AGVindex; 

arcLength = 0;
for numbersOfSamplesPushed = 0:1999%1:2000
    
    % when endpoint and startpoint match --> close the loop [xq(n),yq(n)]->[xq(0),yq(0)]
    if (indx+1+numbersOfSamplesPushed) > length(xq) && ...
            (xq(1) - xq(end)) > 0.01 && ...
            (yq(1) - yq(end)) > 0.01
        indx = indx - length(xq) + 1;
    end
    
    if numbersOfSamplesPushed == 0%1  
        % first integration from closest point xy to [xq(a),yq(a)]
        arcLength = sqrt( (xq(indx+1)-xy(1))^2 + (yq(indx+1)-xy(2))^2 );
    else 
%         if (indx+1+numbersOfSamplesPushed) < length(xq) && targetFound == false
            if (indx+1+numbersOfSamplesPushed) < xIDindx || (indx+1+numbersOfSamplesPushed) < yIDindx
                % continue integration from [xq(a),yq(a)]->[xq(a+1),yq(a+1)]
                arcLength = arcLength + dq(indx+numbersOfSamplesPushed);
               % arcLength = arcLength + sqrt( (xq(indx+1+numbersOfSamplesPushed) - xq(indx+1+(numbersOfSamplesPushed-1)))^2 + (yq(indx+1+numbersOfSamplesPushed) - yq(indx+1+(numbersOfSamplesPushed-1)))^2);
            else
%                 targetFound = true; 
                numbersOfSamplesPushed = xIDindx - indx;
                break; 
            end
%         end
    end
    if (indx+1+numbersOfSamplesPushed) > (xIDindx - 1) % == length(xq) % [m] 
        numbersOfSamplesPushed = xIDindx - indx;
        break;
    elseif arcLength > targetLength 
        break;
    end  
end