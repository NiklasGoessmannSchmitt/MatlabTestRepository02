function targetComp = targetToleranceComp(target, tolerance) 
% targetComp = targetToleranceComp(target, tolerance) 

targetComp = target; 

if abs(target(1)) < tolerance(1)
    targetComp(1) = 0; 
end    

if abs(target(2)) < tolerance(2)
    targetComp(2) = 0; 
end   

if abs(target(3)) < tolerance(3)
    targetComp(3) = 0; 
end   

end