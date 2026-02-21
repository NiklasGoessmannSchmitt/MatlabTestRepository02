function [numbersOfSamplesPushed, arcLength] = pushTargetOnTrack2(xq,yq, dq, xIDindx,yIDindx,AGVindex, xy, targetLength)


f=@(t)abs(sum(dq(AGVindex:(round(AGVindex+t))))-targetLength);

numbersOfSamplesPushed = round(fminbnd(f,0,xIDindx-AGVindex));

arcLength=sum(dq(AGVindex:(AGVindex+numbersOfSamplesPushed)));

end

%% andere kurven in bezier ausgeben

%% monitorig cases export