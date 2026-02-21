function xy = distance2beztest(b,xAGV,yAGV)
curvesegments = length(b);
distf = cell (1,curvesegments);
pick= ones(1,curvesegments);
dista = 500*pick; % a high number so that the entries that are not calculated are not found by the min solver
persistent lastsegmentreached

for i= 1:curvesegments
     %dist is a function of the pythagoras between current position and the bezier segments:
     distf{i}=@(t) sqrt((((1-t).^2*b{i}.x(1) + 2*(1-t).*t*b{i}.x(2) + t.^2*b{i}.x(3))-xAGV)^2 + (((1-t).^2*b{i}.y(1) + 2*(1-t).*t*b{i}.y(2) + t.^2*b{i}.y(3)) - yAGV)^2);
     
     pick(i)=distf{i}(0);   %distance to start point of every segment 
end

if curvesegments == 1 
    %look for minimum in bezier functions between t=0 and t=1
    [t(1),dista(1)]=fminbnd(distf{1},0,1);
    q = 1; 
else
    
    [~, q] = min(pick); %find which segment has the closest startpoint
    
    if q > 1 %take the segment before as well
        r=q-1;
    else %if start is first only take first
        r=1;
    end

    if lastsegmentreached
        for i=1:(curvesegments-1):curvesegments
        %look for minimum in bezier functions between t=0 and t=1
        [t(i),dista(i)]=fminbnd(distf{i},0,1);
        end
    else
        
        for i=r:q
        %look for minimum in bezier functions between t=0 and t=1
        [t(i),dista(i)]=fminbnd(distf{i},0,1);
        end
    end
    
    [~, q] = min(dista); %find minimum of the two considered segments
        
end  

% LOOP CLOSURE
if q == curvesegments
    lastsegmentreached =1;
else
    lastsegmentreached=0;
end

% xy
t=t(q);
xy(1)=(1-t)^2*b{q}.x(1) + 2*(1-t)*t*b{q}.x(2) + t^2*b{q}.x(3);
xy(2)=(1-t)^2*b{q}.y(1) + 2*(1-t)*t*b{q}.y(2) + t^2*b{q}.y(3);
end