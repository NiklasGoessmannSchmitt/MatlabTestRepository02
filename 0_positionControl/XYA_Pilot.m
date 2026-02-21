function v = XYA_Pilot(target, pos, angleOffset, mode, targetSpeed, interimTargetLength, finalTargetLength) 

persistent angleSetpoint
persistent statDistanceSetpoint_old
if isempty(angleSetpoint) 
    angleSetpoint = 0; 
    statDistanceSetpoint_old = zeros(3,1);
end

%% INIT
distanceDeviation = target - statDistanceSetpoint_old; 
if ( sqrt( distanceDeviation(1)^2 + distanceDeviation(2)^2 ) > 0.1 ... % [m]
    || abs(distanceDeviation(3) ) > 0.1 ) %% [rad]
    
    angleSetpoint = pos(3); 
    
end
statDistanceSetpoint_old = target;

%%
if abs(target(1))>0 || abs(target(2))>0 || abs(target(3))>0  
    if mode == 0 
        v(1) = 0.45 * target(1); 
        v(2) = 0.45 * target(2);
        v(3) = 0.75 *(target(3) + angleOffset);

    elseif mode == 1
        v(1) = 0.4 * target(1); 
        v(2) = 0.4 * target(2); 
        v(3) = 1.0 *(angleSetpoint - pos(3)); 
    end 
    
else
    v(1) = 0; 
    v(2) = 0; 
    v(3) = 0; 
end 
    
v = v.*(finalTargetLength/interimTargetLength); % scale up to very high numbers
tempSpeed = sqrt( v(1)^2 + v(2)^2 );
if tempSpeed > targetSpeed
    v = v.*(targetSpeed/tempSpeed); % scale down to target speed
end