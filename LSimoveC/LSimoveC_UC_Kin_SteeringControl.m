function SetpointSteeringVelocity = LSimoveC_UC_Kin_SteeringControl(Release, AngleSetpoint, AngleCurrent, ...
    SteerShortestWay, iOmegaMaxRadS, iMaxAccelerationWheel, iKp, SamplingTime)
% SetpointSteeringVelocity = LSimoveC_UC_Kin_SteeringControl
%       (Release, AngleSetpoint, AngleCurrent, SteerShortestWay, iOmegaMaxRadS, iMaxAccelerationWheel, iKp, SamplingTime)

persistent statSetpointSteeringSpeed
if isempty(statSetpointSteeringSpeed)
    statSetpointSteeringSpeed = 0;
end

tempControlDeviation = AngleSetpoint - AngleCurrent;
% Determine shortest way for steering
if tempControlDeviation >= 0
    if tempControlDeviation > pi && SteerShortestWay 
        tempSteeringDirection = -1;
        tempControlDeviation = 2 * pi - tempControlDeviation;
    else
        tempSteeringDirection = 1;
    end
    % Control deviation negative
else
    if tempControlDeviation < -pi && SteerShortestWay 
        tempSteeringDirection = 1;
        tempControlDeviation = 2 * pi + tempControlDeviation;
    else
        tempSteeringDirection = -1;
        tempControlDeviation = - tempControlDeviation;
    end
end

% Check if control deviation exceeds tolerance
tempSteeringSpeed = 0;
if Release 
    % Proportional part
    tempProportional = iKp * tempSteeringDirection * tempControlDeviation;
    tempSteeringSpeed = tempProportional / SamplingTime;
end

% Truncate to maximum speed
if tempSteeringSpeed > 0 && tempSteeringSpeed > iOmegaMaxRadS 
    tempSteeringSpeed = iOmegaMaxRadS;
elseif tempSteeringSpeed < 0 && abs(tempSteeringSpeed) > iOmegaMaxRadS 
    tempSteeringSpeed = - iOmegaMaxRadS;
end

statSetpointSteeringSpeed = fbRampe(tempSteeringSpeed, ...
                                    statSetpointSteeringSpeed, ...
                                    iMaxAccelerationWheel, ...
                                    iMaxAccelerationWheel, ...
                                    SamplingTime);

SetpointSteeringVelocity = statSetpointSteeringSpeed;