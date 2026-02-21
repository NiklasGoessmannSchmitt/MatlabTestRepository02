function [SetpointSteeringVelocity, RampingFactor] = LSimoveC_UC_MoveCtrl_NP_SteeringControl(n, AngleSetpoint, Release, iKp, PivotValues, iOmegaMaxRadS, iMaxAccelerationWheel, SamplingTime)
% Steering control
% This function block generates a target steering angle depending on target angle and actual angle using a PID controller for each wheel individually.
% A maximum steering speed can be defined which will not be exceeded. 
% This maximum speed can be changed on the fly - but the output speed will not be truncated immediately! 
% A ramp function will reduce output speed with respect to the defined acceleration. I.e. there will always be a delay!
% 
% Shortest way: This function block consists of a function to steer shortest way (only activate if its mechanically possible!!!)
%    E.g.: Deviation 90°  --> Shortest way anticlockwise
%          Deviation 181° --> Shortest way clockwise
%    Attention: This function may cause troubles if the target angles change dramatically during vehicle movement
%       (changes in target angles do always to change continuously in a speed that can be followed by the actual angle!)
%          
% Author: DaG
%  29.03.2020: Insertion of Common Ramping of Steering Angles
%              Insertion of Common Wheel Velocity reduction about a reducing factor 
% *)

persistent statAngleSetpointOld 
persistent statTargetDeltaAngle
persistent SetpointSteeringVelocityOld

if isempty(statAngleSetpointOld)
    statAngleSetpointOld = zeros(1,n); 
    statTargetDeltaAngle = zeros(1,n);
    SetpointSteeringVelocityOld = zeros(1,n);
end

tempControlDeviation = zeros(1,n);
tempSteeringDirection = zeros(1,n);
tempTargetDeltaAngle = zeros(1,n);
tempPartialAngleCorrection = zeros(1,n);
SetpointSteeringVelocity = zeros(1,n);

for tempWheelCounter = 1:n

    tempControlDeviation(tempWheelCounter) = AngleSetpoint(tempWheelCounter) - statAngleSetpointOld(tempWheelCounter);
    
    forbiddenAngles(1).higher = 0; 
    forbiddenAngles(1).lower = 0; 
    forbiddenAngles(2).higher = 0;
    forbiddenAngles(2).lower = 0; 
    [posSteeringAllowed, negSteeringAllowed] = LSimoveC_Internal_CheckSteeringDirection(statAngleSetpointOld(tempWheelCounter), ... %actualAngle,
                                                                                        AngleSetpoint(tempWheelCounter), ... % targetAngle
                                                                                        forbiddenAngles);

    % Determine shortest way for steering
    if (tempControlDeviation(tempWheelCounter) >= 0.0) 

       if (tempControlDeviation(tempWheelCounter) > pi) && negSteeringAllowed 
          tempSteeringDirection(tempWheelCounter) = -1;
          tempControlDeviation(tempWheelCounter) = 2*pi - tempControlDeviation(tempWheelCounter);
       else
           tempSteeringDirection(tempWheelCounter) = 1;
       end
       % Control deviation negative
    else

       if (tempControlDeviation(tempWheelCounter) < -pi) && posSteeringAllowed 
          tempSteeringDirection(tempWheelCounter) = 1;
          tempControlDeviation(tempWheelCounter) = 2*pi + tempControlDeviation(tempWheelCounter);
       else
           tempSteeringDirection(tempWheelCounter) = -1;
           tempControlDeviation(tempWheelCounter) = -tempControlDeviation(tempWheelCounter);
       end
    end
    
    if Release 

       tempTargetDeltaAngle(tempWheelCounter) = fbRampe(...
           tempSteeringDirection(tempWheelCounter) * tempControlDeviation(tempWheelCounter), ...
           statTargetDeltaAngle(tempWheelCounter), ...
           iMaxAccelerationWheel, ...
           iMaxAccelerationWheel, ...
           SamplingTime); 

       if tempTargetDeltaAngle(tempWheelCounter) ~= (tempSteeringDirection(tempWheelCounter)*tempControlDeviation(tempWheelCounter))
           ;
       end

       statTargetDeltaAngle(tempWheelCounter) = tempTargetDeltaAngle(tempWheelCounter); 
       if abs(tempControlDeviation(tempWheelCounter)) > 1e-6 
          tempPartialAngleCorrection(tempWheelCounter) = tempTargetDeltaAngle(tempWheelCounter) / (tempSteeringDirection(tempWheelCounter) * tempControlDeviation(tempWheelCounter));
       else
          tempPartialAngleCorrection(tempWheelCounter) = 1.0;
       end
    end
end
 
% Find out smallest step
tempRampingFactor = 1; 
for tempWheelCounter = 1:n
    if tempPartialAngleCorrection(tempWheelCounter) < tempRampingFactor 
       tempRampingFactor = tempPartialAngleCorrection(tempWheelCounter);
    end
end
 
% Use calculated ramping factor as factor for deviation
for tempWheelCounter = 1:n
    if Release
        tempDeltaAngleSetpoint(tempWheelCounter) = tempSteeringDirection(tempWheelCounter) * tempControlDeviation(tempWheelCounter) * tempRampingFactor;
        statAngleSetpoint(tempWheelCounter) = tempDeltaAngleSetpoint(tempWheelCounter) + statAngleSetpointOld(tempWheelCounter);
        tempAngleDeviation(tempWheelCounter) = statAngleSetpoint(tempWheelCounter) - PivotValues(tempWheelCounter).angularPosition;
        tempSteeringSpeed = iKp * tempAngleDeviation(tempWheelCounter) / SamplingTime;
   
         % Truncate to maximum speed
        if tempSteeringSpeed > 0.0 && tempSteeringSpeed > iOmegaMaxRadS 
             tempSteeringSpeed = iOmegaMaxRadS;
        elseif tempSteeringSpeed < 0.0 && abs(tempSteeringSpeed) > iOmegaMaxRadS 
            tempSteeringSpeed = -iOmegaMaxRadS;
         end
        SetpointSteeringVelocity(tempWheelCounter) = tempSteeringSpeed;
    else
        SetpointSteeringVelocity(tempWheelCounter) = 0;
    end

    SetpointSteeringVelocity(tempWheelCounter) = fbRampe(SetpointSteeringVelocity(tempWheelCounter), SetpointSteeringVelocityOld(tempWheelCounter), iMaxAccelerationWheel, iMaxAccelerationWheel, SamplingTime);
end

if Release
   statAngleSetpointOld = statAngleSetpoint;
   SetpointSteeringVelocityOld = SetpointSteeringVelocity;
else
   % If no release: write actual values to the outputs
   for tempWheelCounter = 1:n 
      statAngleSetpointOld(tempWheelCounter) = PivotValues(tempWheelCounter).AngularPosition;
   end
end
RampingFactor = tempRampingFactor;