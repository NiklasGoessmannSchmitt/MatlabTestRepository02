function xy = distance2bezier3(b,xAGV,yAGV)

curvesegments = length(b); %number of bezier elements

if curvesegments == 0
    
    error('no bezier elements in track')
    
else
    
    distf = cell (1,curvesegments);
    pick= ones(1,curvesegments);
    dista = 500*pick; %a high number so that the entries that are not calculated are not found by the min solver
    a =0;
    persistent oldsegment
    
    %% a vector should contain the bezier elements before and after the current one
    if (oldsegment > 1) & (oldsegment ~= curvesegments)
        
        a = oldsegment -1 :1: oldsegment +1;
        
    elseif (oldsegment == 1) & (1 ~= curvesegments)
        
        a = [1 2 curvesegments];
        
    elseif (oldsegment == curvesegments ) & (1 ~= curvesegments)
        
        a = [curvesegments-1 curvesegments 1];
        
    else
        
        a= 1;
        
    end
    
    %% finding minimum distance between the chosen bezier elements and the AGV position
    for n = 1:length(a)
        
        %dist is a function of pythagoras between current position and the bezier segments:
        i = a(n);
        distf{i}=@(t) sqrt((((1-t).^2*b{i}.x(1) + 2*(1-t).*t*b{i}.x(2) + t.^2*b{i}.x(3))-xAGV)^2 + (((1-t).^2*b{i}.y(1) + 2*(1-t).*t*b{i}.y(2) + t.^2*b{i}.y(3)) - yAGV)^2);
        
        [t(i),dista(i)]=fminbnd(distf{i},0,1); %min solver between t=[0 1]. dista contains distance values
        
    end

[~, q] = min(dista); %find minimum of the considered segments
oldsegment = q;
t=t(q);

%xy values for AGV mapping
xy(1)=(1-t)^2*b{q}.x(1) + 2*(1-t)*t*b{q}.x(2) + t^2*b{q}.x(3);
xy(2)=(1-t)^2*b{q}.y(1) + 2*(1-t)*t*b{q}.y(2) + t^2*b{q}.y(3);

end
end