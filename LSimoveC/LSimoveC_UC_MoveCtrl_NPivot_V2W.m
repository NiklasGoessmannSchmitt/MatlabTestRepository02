function [oSetpointVelocities, oSetpointAlpha, oReducingFactor, ioV, ioError] = LSimoveC_UC_MoveCtrl_NPivot_V2W(...
        vXYA, pivotValues, pivotParam ,n, iOmegaMaxRadS, RampingFactorSteering, tolerance, toleranceDriving, iStandstill, rotationOnly) 

%  Description
%  Forward Kinematics - holonomic
%  In this function block a angular setpoint and a wheel velocity is 
%  calculated based on the vehicle setpoint velocity and the wheel's 
%  position in the AGV.
% 
%  Version
%  V1.0 2012-03-19 DG: First Version
%  V1.1 2020-04-01 DG: Reducing wheel velocity due to time delay caused by steering
%  V1.2 2020-04-07 DG: Remove statLastAngleCommand, instead: statLastValidAngleOutputs
% 
%  Autors
%  DG: Daniel Gauglitz
% 
%  © Alle Rechte vorbehalten/All rights reserved, Siemens AG, 2018

persistent statDrivingRequest
persistent statValuesInverted
persistent statSetpointRotationalVelocityOld
persistent statDirectionInvertionOverlain
persistent statTargetDirectionInvertionOverlain
persistent statRqStop
persistent statError
persistent statMovement
persistent statRotationOnly

if isempty(statDrivingRequest)
    statDrivingRequest = false; 
    statValuesInverted = zeros(1,n); 
    statSetpointRotationalVelocityOld = 0;
    statDirectionInvertionOverlain = zeros(1,n);
    statTargetDirectionInvertionOverlain = zeros(1,n); 
    statRqStop = false;
    statError = false;
    statMovement = false; 
    statRotationOnly = false;
end

% statError = ioError;
tempErrorAmountWheels = false; %("MAX_INDEX_AXES" < 1);

%% Initialisierung  
tempDrivingRequest = false;
tempSteering = 0;
tempReducingFactor = 1.0;
tempReducingFactorRobot = 1.0;
tempIsReduced = false;
tempVMaxMS = 99999999;%vMax;     
tempOmegaCenterRadS = vXYA(3);
tempVCenterXMS = vXYA(1);
tempVCenterYMS = vXYA(2);
ioError = false;
oSetpointVelocities = zeros(1,n);
oSetpointAlpha = zeros(1,n);
tempErrorReduceCounter = false;

%% Generation of driving request
% setting the vehicle into movement requires each wheel to be in the right angle
if abs(tempOmegaCenterRadS) > 0 || abs(tempVCenterXMS) > 0 || abs(tempVCenterYMS) > 0 
    tempDrivingRequest = true;
end

if tempDrivingRequest == true && statDrivingRequest == false 
    statMovement = false;
end

statDrivingRequest = tempDrivingRequest; 

%% Reducing factor for X Y
% limit command values at once!
% if the combination of command values is resulting in a higher speed per wheel a reducing factor is multiplied later within calculation of each wheel
tempVCompMaxMS = max(abs(tempVCenterXMS), abs(tempVCenterYMS));

% check if v_x or v_y exceed given limitation
if tempVCompMaxMS > tempVMaxMS 

    tempReducingFactor = min(1.0, abs(tempVMaxMS / tempVCompMaxMS));
    tempReducingFactorRobot = tempReducingFactor;

    % limit variables for internal use
    tempVCenterXMS = tempReducingFactor * tempVCenterXMS;
    tempVCenterYMS = tempReducingFactor * tempVCenterYMS;
    tempOmegaCenterRadS = tempReducingFactor * tempOmegaCenterRadS;

    % reduce inouts
    ioV(3) = tempOmegaCenterRadS; % reduce inout variable
    ioV(1) = tempVCenterXMS * 1000.0; % reduce inout variable
    ioV(2) = tempVCenterYMS * 1000.0; % reduce inout variable

    tempIsReduced = true;
end


%% Reducing factor Omega
%check if omega exceeds given limitation (wird ausgeführt wenn Omega auch nach der Reduzierung nach VCenter noch größer als MaxOmega oder wenn nur Omega größer MaxOmega)
if abs(tempOmegaCenterRadS) > iOmegaMaxRadS 

    tempReducingFactor = min(1.0, abs(iOmegaMaxRadS / tempOmegaCenterRadS));

    % sum up reducing factors (omega + v_center) -> multiplication
    tempReducingFactorRobot = tempReducingFactorRobot * tempReducingFactor;

    % limit variables for internal use
    tempVCenterXMS = tempReducingFactor * tempVCenterXMS;
    tempVCenterYMS = tempReducingFactor * tempVCenterYMS;

    tempOmegaCenterRadS = tempReducingFactor * tempOmegaCenterRadS;

    % reduce inouts
    ioV(3) = tempOmegaCenterRadS;
    ioV(1) = tempVCenterXMS * 1000.0;
    ioV(2) = tempVCenterYMS * 1000.0;

    tempIsReduced = true;
end

% init number of reduced wheels
tempReduceCounter = 0;

% init reducing factor to 1.0 again for further use, its old value is saved in reducingFactor_robot
tempReducingFactor = 1.0;
tempReducingFactorWheels = 1.0;

%% RecalculationTractionSpeedWithReducedSpeed
% loop point for recalculation of the traction setpoints with reduced speed
for a = 1:10 % NOT #tempReducingAgain
    % if in reducing mode, use the reducingFactor; max trials = amountWheels
   if tempReduceCounter >= (n-1 + 5) || (tempReduceCounter < 0) %  +5, due to debug reasons

        % if this happens maybe something is wrong with wheel configuration
        ioError = true;
        tempErrorReduceCounter = true;

   elseif tempReduceCounter > 0 

        %limit variables for internal use
        tempVCenterXMS = tempVCenterXMS * tempReducingFactor;
        tempVCenterYMS = tempVCenterYMS * tempReducingFactor;

        tempOmegaCenterRadS = tempOmegaCenterRadS * tempReducingFactor;

        tempIsReduced = true;
        tempErrorReduceCounter = false;
   end

     % Calc deviation of rotational velocity
     tempDeviationRotationalVelocity = tempOmegaCenterRadS - statSetpointRotationalVelocityOld;

    % calcluate setpoints for each wheel    
    for tempWheelCounter = 1:n 

       %wheel configuration
       tempDistXM = pivotParam(tempWheelCounter).position.x; % [m]
       tempDistYM = pivotParam(tempWheelCounter).position.y; % [m]
       tempPhiRad = 0.0; % [rad]

        % calculate velocity of the current wheel
        tempVWheelXMS = tempVCenterXMS - (tempOmegaCenterRadS * tempDistYM);
        tempVWheelYMS = tempVCenterYMS + (tempOmegaCenterRadS * tempDistXM);

        % norm this velocity (euklidean length) to get the setpoint for the drive
        tempVWheelMS = sqrt(tempVWheelXMS * tempVWheelXMS + tempVWheelYMS * tempVWheelYMS);

        % check if this traction setpoint exceeds the maximum
        if abs(tempVWheelMS) > tempVMaxMS 

            tempReduceCounter = tempReduceCounter + 1;

            % reducing factor has to be positive ]0.0, 1.0]
            tempReducingFactor = min(1.0, abs(tempVMaxMS / tempVWheelMS));

            % sum up reducing factors (v_wheel[i]) -> multiplication
            tempReducingFactorWheels = tempReducingFactor * tempReducingFactorWheels;

            % -->  reCalc all wheels with reduced velocity 
            tempReducingAgain = true;
        else
            tempReducingAgain = false;
        end

        if tempReducingAgain
            break;
        end

        % get steering angle of this wheel from velocity
        if abs(tempVWheelMS) > 0.0 
            tempPhiRad = atan2(tempVWheelYMS, tempVWheelXMS);

        else
           % do nothing when wheel has a velocity setpoint which equals zeros
%            tempPhiRad = tempPhiRad;
           % phi act could be within [0;2pi], we want always the same intervall
           % Util_NormAngle" normalizes the angle in range [-pi; pi]
           tempPhiRad = normAngle(tempPhiRad);
        end
        tempSetpointAlpha = tempPhiRad;
        
        % Correct VWheel about Steering Delay
        tempVWheelXMS = tempVCenterXMS - ...
                    (statSetpointRotationalVelocityOld + tempDeviationRotationalVelocity * RampingFactorSteering) * tempDistYM;

        tempVWheelYMS = tempVCenterYMS + ...
                    (statSetpointRotationalVelocityOld + tempDeviationRotationalVelocity * RampingFactorSteering) * tempDistXM;

        tempVWheelMS = sqrt(tempVWheelXMS^2 + tempVWheelYMS^2);

        % write data (velocity and steering angle) back into wheel db
        % set setpoint values to zero, if no movement is required  
        if ~statMovement 
            tempSetpointVs = 0;
        else
           % invert speed and turn steering angle about 180° if driving direction is backward 
           tempSetpointVs = tempVWheelMS;% * 1000.0;
        end

       % invert speed and turn steering angle about 180° per wheel if driving steering angle is not allowed
       if statDirectionInvertionOverlain(tempWheelCounter) 
         if tempSetpointAlpha >= 0 
             tempSetpointAlpha = mod((tempSetpointAlpha - pi), (2*pi));
         else
             tempSetpointAlpha = mod((tempSetpointAlpha + pi), (2*pi));
         end
          tempSetpointVs = tempSetpointVs* -1;
       end
        
       %% Write signal for invertion request (1) 
       if pivotParam(tempWheelCounter).forbiddenAngles(1).higher - pivotParam(tempWheelCounter).forbiddenAngles(1).lower > 0 && ...
               tempSetpointAlpha > pivotParam(tempWheelCounter).forbiddenAngles(1).lower && ... 
               tempSetpointAlpha <= pivotParam(tempWheelCounter).forbiddenAngles(1).higher
           statTargetDirectionInvertionOverlain(tempWheelCounter) = ~statDirectionInvertionOverlain(tempWheelCounter); 
       elseif pivotParam(tempWheelCounter).forbiddenAngles(2).higher - pivotParam(tempWheelCounter).forbiddenAngles(2).lower > 0 && ...
               tempSetpointAlpha > pivotParam(tempWheelCounter).forbiddenAngles(2).lower && ...
               tempSetpointAlpha <= pivotParam(tempWheelCounter).forbiddenAngles(2).higher 
           statTargetDirectionInvertionOverlain(tempWheelCounter) = ~statDirectionInvertionOverlain(tempWheelCounter);
       end
       %%
       
       tempDiffAngle = tempSetpointAlpha - pivotValues(tempWheelCounter).angularPosition;
       %% REGION Write signal for invertion request
       % Check which steering directions are allowed
      [tempPositiveSteeringValid(tempWheelCounter), tempNegativeSteeringValid(tempWheelCounter)] = LSimoveC_Internal_CheckSteeringDirection...
          (pivotValues(tempWheelCounter).angularPosition, ...
          tempSetpointAlpha, ...
          pivotParam(tempWheelCounter).forbiddenAngles);

      if ~statMovement && tempDrivingRequest && ...
         statTargetDirectionInvertionOverlain(tempWheelCounter) == statDirectionInvertionOverlain(tempWheelCounter) 
         if (tempDiffAngle >= 0.0) 
            if ((tempDiffAngle > pi) && tempNegativeSteeringValid(tempWheelCounter)) || ...
               ((tempDiffAngle <= pi) && tempNegativeSteeringValid(tempWheelCounter) && ~tempPositiveSteeringValid(tempWheelCounter))

               tempDiffAngle = 2 * pi - tempDiffAngle;
            end

         else % Control deviation negative
            if ((tempDiffAngle < - pi) && tempPositiveSteeringValid(tempWheelCounter)) || ...
               ((tempDiffAngle >= -pi) && tempPositiveSteeringValid(tempWheelCounter) && ~tempNegativeSteeringValid(tempWheelCounter))

                tempDiffAngle = 2*pi + tempDiffAngle;
            end
         end
         % Check stering deviation for inverted angle
         [tempPositiveSteeringValid(tempWheelCounter), tempNegativeSteeringValid(tempWheelCounter)] = LSimoveC_Internal_CheckSteeringDirection...
            (pivotValues(tempWheelCounter).angularPosition, ...
             normAngle(tempSetpointAlpha + pi),  ... 
             pivotParam(tempWheelCounter).forbiddenAngles);

         % Calulate deviation for alternative steering direction
         tempDiffAngleAlternative = normAngle(tempSetpointAlpha + pi) - pivotValues(tempWheelCounter).angularPosition;
         if (tempDiffAngleAlternative >= 0.0) 
            if ((tempDiffAngleAlternative > pi) && tempNegativeSteeringValid(tempWheelCounter)) || ...
               ((tempDiffAngleAlternative <= pi) && tempNegativeSteeringValid(tempWheelCounter) && ~tempPositiveSteeringValid(tempWheelCounter)) 

                tempDiffAngleAlternative = 2*pi - tempDiffAngleAlternative;

            end
         else % Control deviation negative
            if ((tempDiffAngleAlternative < -pi) && tempPositiveSteeringValid(tempWheelCounter)) || ...
               ((tempDiffAngleAlternative >= -pi) && tempPositiveSteeringValid(tempWheelCounter) && ~tempNegativeSteeringValid(tempWheelCounter))

               tempDiffAngleAlternative = 2*pi + tempDiffAngleAlternative;
            end
         end

         if abs(tempDiffAngle) > (pi / 2.0 + tolerance) && ...
            abs(tempDiffAngleAlternative) < abs(tempDiffAngle) && ~statValuesInverted(tempWheelCounter)
            % Check if inverting is not possible (allowed) 
            if pivotParam(tempWheelCounter).forbiddenAngles(1).higher - pivotParam(tempWheelCounter).forbiddenAngles(1).lower > 0 && ...
               normAngle(tempSetpointAlpha + pi) > pivotParam(tempWheelCounter).forbiddenAngles(1).lower && ...
               normAngle(tempSetpointAlpha + pi) <= pivotParam(tempWheelCounter).forbiddenAngles(1).higher || ...
               ...
               pivotParam(tempWheelCounter).forbiddenAngles(2).higher - pivotParam(tempWheelCounter).forbiddenAngles(2).lower > 0 && ...
               normAngle(tempSetpointAlpha + pi) > pivotParam(tempWheelCounter).forbiddenAngles(2).lower && ...
               normAngle(tempSetpointAlpha + pi) <= pivotParam(tempWheelCounter).forbiddenAngles(2).higher || ...
               statTargetDirectionInvertionOverlain(tempWheelCounter) ~= statDirectionInvertionOverlain(tempWheelCounter) 

               % Don't change target angle if not possible to invert
               statTargetDirectionInvertionOverlain(tempWheelCounter) = statTargetDirectionInvertionOverlain(tempWheelCounter);
            else % Invert
                statTargetDirectionInvertionOverlain(tempWheelCounter) = ~statTargetDirectionInvertionOverlain(tempWheelCounter);
                statValuesInverted(tempWheelCounter) = true;
            end
         end
      end

        
        %%
        tempDiffAngle = abs(tempSetpointAlpha - pivotValues(tempWheelCounter).angularPosition);
        if tempDiffAngle > pi 
            tempDiffAngle = abs(tempDiffAngle - (2*pi));
        end
        % steering request is true, if deviation in steering angle is greater than tolerance (for whole vehicle)
        if tempDiffAngle > tolerance && ~statMovement && tempDrivingRequest 
            tempSteering = true;
        end

        if statValuesInverted(tempWheelCounter) && ... 
                (statMovement || (~tempDrivingRequest &&  ~statRqStop))
            statValuesInverted(tempWheelCounter) = false; 
        end
        
        if tempDiffAngle > toleranceDriving && statMovement
            statRqStop = true; 
        end
        
        if rotationOnly ~= statRotationOnly 
            statRqStop = true; 
        end
        statRotationOnly = rotationOnly; 
        
        % Write output values for wheel
        oSetpointVelocities(tempWheelCounter) = tempSetpointVs;
        if isnan(tempSetpointVs) 
            disp('NPivot NaN error!')
        end 
        
        oSetpointAlpha(tempWheelCounter) = tempSetpointAlpha;
    end % wheelCounter
end % NOT #tempReducingAgain
% tempReducingAgain is the expression which is evaluated in every cycle of the loop.
% loop is calculated again if tempReducingAgain is FALSE
% loop is terminated and programm is proceeded after "END_REPEAT" if tempReducingAgain is TRUE

%  Write Rotational velocity for next cycle
statSetpointRotationalVelocityOld = statSetpointRotationalVelocityOld + tempDeviationRotationalVelocity * RampingFactorSteering;


% Set request for standstill if target direction is not actual direction
if statDirectionInvertionOverlain ~= statTargetDirectionInvertionOverlain 
   statRqStop = true;
   % #oSetpointAlpha := #statLastValidAngleOutputs;
else
   statLastValidAngleOutputs = oSetpointAlpha;
end

if statRqStop || ~tempDrivingRequest 
   oSetpointAlpha = statLastValidAngleOutputs;
end
   
 
% Write direction invertion if during standstill
if sum(statDirectionInvertionOverlain ~= statTargetDirectionInvertionOverlain) && iStandstill && ~tempDrivingRequest 
   statDirectionInvertionOverlain = statTargetDirectionInvertionOverlain;
end

% reset stop request during standstill
if iStandstill && ~tempDrivingRequest 
    statRqStop = false;
end

%% ReturnValues
% return reduced values
if tempIsReduced && statMovement 

    % sum up reducing factors (v_center + omega + v_wheel[i]) -> multiplication
    oReducingFactor = tempReducingFactorWheels * tempReducingFactorRobot;

    % set inouts to currently reduced values
    ioV(3) = tempOmegaCenterRadS;
    ioV(1) = tempVCenterXMS * 1000.0;
    ioV(2) = tempVCenterYMS * 1000.0;
else
    oReducingFactor = 1.0;
end

% driving is permitted if steering deviance is within tolerance
if ~statMovement && ~tempSteering && tempDrivingRequest 
    statMovement = true;
end

oSetpointsReduced = tempIsReduced;
oSteering = tempSteering;
oMovement = statMovement;
oRqDrive = tempDrivingRequest;
oRqStop = statRqStop;


%% FEHLERHANDLING 
if (~tempErrorAmountWheels && ~tempErrorReduceCounter && ~statError) 
    %% leave function_SCLConverter block    
else
    if tempErrorAmountWheels 
       ErrorCode = 1;
    elseif tempErrorBlockAtan 
       ErrorCode = 2;
    elseif tempErrorReduceCounter 
       ErrorCode = 3;
    end
    % ERROR HANDLING
    % this is only reached, when an error occured in this function_SCLConverter

    % DEFINITION OF SAVE VALUES
%         (*
%    all speed setpoints are set to zeros (robot brakes to stand still and remains in no move condition)
%    all steering setpoints remain as they are (wheels do not rotate anymore)
%    
%    setpointsReduced := TRUE
%    out_reducingFactor := 0.0 -> when used outside, reduce everything to zero
%    
%    inout_error := TRUE -> this error has to be reset by the application
%   *)
   for tempWheelCounter = 0:(n-1)
        % write data (velocity and steering angle) into wheel db
       oSetpointVelocities(tempWheelCounter) = 0;
       oSetpointAlpha(tempWheelCounter) = statLastValidAngleOutputs(tempWheelCounter);
   end
    ioV(1) = 0;
    ioV(2) = 0;
    ioV(3) = 0; 
    oSetpointsReduced = true;
    ioError = true;
    oReducingFactor = 0.0;
end


if isnan(oSetpointVelocities(1)) || isnan(oSetpointVelocities(2))
    disp('weird error!')
end 
