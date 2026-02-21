function [vS, alpha, steering, movement, stopReq] = LSimoveC_UC_Kin_Tricycle_V2W(v, axesParam, currentAlpha, tolerance, forbiddenAngles, rotationOnly, standstill)
% [vs, alpha, steering, movement, stopRq] = LSimoveC_UC_Kin_Tricycle_V2W(v, axesParam, currentAlpha, angularTolerance)

CONST_STATE_STANDSTILL = 0; 
CONST_STATE_WHEELPOSITIONING = 1; 
CONST_STATE_WHEELPOSITIONING_INV = 2;
CONST_STATE_MOVEMENT = 3; 
CONST_STATE_MOVEMENT_INV = 4; 
CONST_STATE_STOPREQ = 5; 

CONST_INVERSION_NORMAL = 0; 
CONST_INVERSION_INVERT = 1; 
CONST_INVERSION_INVERT_2 = 2; 
CONST_INVERSION_REVERT = 3; 

CONST_NULL_P = 1e-6;

%% Persistent variables
persistent statLastAngle 
persistent statLastVs
persistent statLastVelo
persistent state
persistent stateInversion
persistent statDirectionInvertionOverlain
persistent statTargetDirectionInvertionOverlain
persistent statValuesInverted
persistent statRotationOnly
persistent statLastVelocity
if isempty(statLastAngle) 
    statLastAngle = 0;
    stateInversion = false; 
    statLastVs = 0; 
    statLastVelo = zeros(1,3);
    state = CONST_STATE_STANDSTILL; 
    statDirectionInvertionOverlain = false; 
    statTargetDirectionInvertionOverlain = false; 
    statValuesInverted = false; 
    statRotationOnly = false; 
    statLastVelocity = 0; 
end 


%% tempDriving 
tempDrivingReq = false;
if abs(v(1)) > 0 || abs(v(3)) > 0
    tempDrivingReq = true;
end

%% 	REGION Kinematic calculations
tempSetpointAlpha = 0;
tempSetpointVs = 0;

if abs(v(1)) < CONST_NULL_P && abs(v(3)) < CONST_NULL_P % to avoid atan(0/0)
    tempSetpointAlpha = statLastAngle;
    tempSetpointVs = abs(v(3)) * axesParam.position.x;
    
elseif abs(v(1)) < CONST_NULL_P && abs(v(3)) > 0 % to avoid atan(x/0)
    
    if v(3) < 0 
        tempSetpointAlpha = - pi/2;
    elseif v(3) > 0
        tempSetpointAlpha = pi/2;
    end
    tempSetpointVs = abs(v(3)) * axesParam.position.x;
    
else % for valid atan values
    
    tempSetpointAlpha = atan(v(3) * axesParam.position.x / v(1)); % in [rad]
    tempSetpointVs = v(1) / cos(tempSetpointAlpha);     %  in [mm/s]
    
end
	
%%	REGION decentralized traction wheel 
	    
if abs(tempSetpointAlpha) > CONST_NULL_P && axesParam.position.y ~= 0
    
    tempAlphaNew = atan(axesParam.position.x / (axesParam.position.x / tan(tempSetpointAlpha) - axesParam.position.y));
    
    if axesParam.position.y < 0 
        if tempSetpointAlpha < atan(axesParam.position.x / axesParam.position.y) 
            tempAlphaNew = tempAlphaNew - pi;
        end
    elseif axesParam.position.y > 0 
        if tempSetpointAlpha > atan(axesParam.position.x / axesParam.position.y)
            tempAlphaNew = tempAlphaNew + pi;
        end
    end
    
    tempSetpointVs = tempSetpointVs * sin(tempSetpointAlpha) / sin(tempAlphaNew);
    tempSetpointAlpha = tempAlphaNew;
end

	
%%	REGION shortest way 
% invert speed and turn steering angle about 180°
if statDirectionInvertionOverlain
    if tempSetpointAlpha >= 0
        tempSetpointAlpha = mod((tempSetpointAlpha - pi), (2 * pi));
    else
        tempSetpointAlpha = mod((tempSetpointAlpha + pi), (2 * pi));
    end
    tempSetpointVs = -1*tempSetpointVs;
end
	    
%%	REGION Evaluate signal for invertion request
if statTargetDirectionInvertionOverlain == statDirectionInvertionOverlain ...
    && ~(state == CONST_STATE_STOPREQ) ...
    && tempDrivingReq

    % check if target angle is within forbidden range
    if forbiddenAngles(1).higher - forbiddenAngles(1).lower > 0 && ...
        tempSetpointAlpha > forbiddenAngles(1).lower && ...
        tempSetpointAlpha <= forbiddenAngles(1).higher

        statTargetDirectionInvertionOverlain = ~statDirectionInvertionOverlain;
    elseif forbiddenAngles(2).higher - forbiddenAngles(2).lower > 0 && ...
        tempSetpointAlpha > forbiddenAngles(2).lower && ...
        tempSetpointAlpha <= forbiddenAngles(2).higher 

        statTargetDirectionInvertionOverlain = ~statDirectionInvertionOverlain;
    end
end
	    
 %% Determine shortest way
tempDiffAngle = (tempSetpointAlpha - currentAlpha);
% only determine shortest way if standstill 
% valid in two cases:
%  - AGV has been standing still
%  - AGV clouldn't find a common steering direction for all wheels --> needs to stop in order to turn

%%	    REGION Write signal for invertion request
% Check which steering directions are allowed
[tempPositiveSteeringValid, tempNegativeSteeringValid] = LSimoveC_Internal_CheckSteeringDirection(currentAlpha, ...
                                           tempSetpointAlpha, ...
                                           forbiddenAngles);

if state == CONST_STATE_WHEELPOSITIONING || state == CONST_STATE_STOPREQ && ...
    statTargetDirectionInvertionOverlain == statDirectionInvertionOverlain 
    if (tempDiffAngle >= 0.0) 
        if (tempDiffAngle > pi) && tempNegativeSteeringValid || ...
            (tempDiffAngle <= pi) && tempNegativeSteeringValid && ~tempPositiveSteeringValid 

            tempDiffAngle = 2 * pi - tempDiffAngle;
        end
        
    else % Control deviation negative
        if (tempDiffAngle < - pi) && tempPositiveSteeringValid || ...
            (tempDiffAngle >= - pi) && tempPositiveSteeringValid && ~tempNegativeSteeringValid
            tempDiffAngle = 2 * pi + tempDiffAngle;
        end
    end
    % Check steering deviation for inverted angle
    [tempPositiveSteeringValid, tempNegativeSteeringValid] = LSimoveC_Internal_CheckSteeringDirection(currentAlpha, ...
                                               normAngle(tempSetpointAlpha + pi), ...
                                               forbiddenAngles);

    % Calulate deviation for alternative steering direction
    tempDiffAngleAlternative = normAngle(tempSetpointAlpha + pi) - currentAlpha;
    if (tempDiffAngleAlternative >= 0.0) 
        if (tempDiffAngleAlternative > pi) && tempNegativeSteeringValid || ...
            (tempDiffAngleAlternative <= pi) && tempNegativeSteeringValid && ~tempPositiveSteeringValid

            tempDiffAngleAlternative = 2 * pi - tempDiffAngleAlternative;
            
        end
    else % Control deviation negative
        if (tempDiffAngleAlternative < - pi) && tempPositiveSteeringValid || ...
            (tempDiffAngleAlternative >= - pi) && tempPositiveSteeringValid && ~tempNegativeSteeringValid

            tempDiffAngleAlternative = 2 * pi + tempDiffAngleAlternative;	                    
        end
    end
    
    if abs(tempDiffAngle) > (pi / 2 + tolerance) && ...
        abs(tempDiffAngleAlternative) < abs(tempDiffAngle) && ~statValuesInverted
        % Check if inverting is not possible (allowed) 
        if forbiddenAngles(1).higher - forbiddenAngles(1).lower > 0 && ...
            normAngle(tempSetpointAlpha + pi) > forbiddenAngles(1).lower && ...
            normAngle(tempSetpointAlpha + pi) <= forbiddenAngles(1).higher || ...
            forbiddenAngles(2).higher - forbiddenAngles(2).lower > 0 && ... 
            normAngle(tempSetpointAlpha + pi) > forbiddenAngles(2).lower && ... 
            normAngle(tempSetpointAlpha + pi) <= forbiddenAngles(2).higher || ...
            statTargetDirectionInvertionOverlain ~= statDirectionInvertionOverlain

            % Don't change target angle if not possible to invert
            statTargetDirectionInvertionOverlain = statTargetDirectionInvertionOverlain;
        else % Invert
            statTargetDirectionInvertionOverlain = ~statTargetDirectionInvertionOverlain;
            statValuesInverted = true;
        end
    end
end

%% End Insert determine shortest way  

if statValuesInverted && ...
    (state == CONST_STATE_MOVEMENT || state == CONST_STATE_STANDSTILL)

    statValuesInverted = false;
end
	    
	
%%	REGION state machine
	    
% Write direction invertion if during standstill
if statDirectionInvertionOverlain ~= statTargetDirectionInvertionOverlain && standstill && ~tempDrivingReq
    statDirectionInvertionOverlain = statTargetDirectionInvertionOverlain;
end

switch state 
    case CONST_STATE_STANDSTILL
        steering = false;
        movement = false;
        stopReq = false;
%         status = CONST_STATE_STANDSTILL;
        
        vS = 0.0;
        alpha = statLastAngle;
        
        if rotationOnly ~= statRotationOnly
            state = CONST_STATE_STOPREQ;
%             status = CONST_MSG_ROTATION_ONLY;
            
        elseif statDirectionInvertionOverlain ~= statTargetDirectionInvertionOverlain 
            state = CONST_STATE_STOPREQ;
%             status = CONST_MSG_DIRECTION_INVERSION;
            
        elseif tempDrivingReq 
            state = CONST_STATE_WHEELPOSITIONING;
%             status = CONST_MSG_NO_DRIVING_REQ;
            
        end
        
    case CONST_STATE_WHEELPOSITIONING
        steering = true;
        movement = false;
        stopReq = false;
%         status = CONST_STATE_WHEELPOSITIONING;
        
        vS = 0.0;
        alpha = tempSetpointAlpha;
        statLastAngle = currentAlpha;
        
        if statDirectionInvertionOverlain ~= statTargetDirectionInvertionOverlain 
            state = CONST_STATE_STOPREQ;
%             status = CONST_MSG_DIRECTION_INVERSION;
            
        elseif rotationOnly ~= statRotationOnly 
            state = CONST_STATE_STOPREQ;
%             status = CONST_MSG_ROTATION_ONLY;
            
        elseif ~tempDrivingReq 
            state = CONST_STATE_STANDSTILL;
%             status = CONST_MSG_NO_DRIVING_REQ;
            
        elseif abs(normAngle(tempSetpointAlpha - currentAlpha)) < tolerance %...
%         && abs(currentSteeringVelocity) < toleranceSteeringVelocity

            state = CONST_STATE_MOVEMENT;
%             status = CONST_MSG_ANGLE_IN_TOLERANCE;
            
        end
        
    case CONST_STATE_MOVEMENT
        steering = false;
        movement = true;
        stopReq = false;
%         status = CONST_STATE_MOVEMENT;
        
        vS = tempSetpointVs;
        alpha = tempSetpointAlpha;
        statLastAngle = currentAlpha;
        
        if statDirectionInvertionOverlain ~= statTargetDirectionInvertionOverlain 
            state = CONST_STATE_STOPREQ;
%             status = CONST_MSG_DIRECTION_INVERSION;
            
        elseif abs(normAngle(tempSetpointAlpha - currentAlpha)) > pi/2 %toleranceDriving 
            state = CONST_STATE_STOPREQ;
%             status = CONST_ERROR_DRIVING_TOLERANCE_EXCEEDED;
            
        elseif rotationOnly ~= statRotationOnly 
            state = CONST_STATE_STOPREQ;
%             status = CONST_MSG_ROTATION_ONLY;
            
        elseif ~tempDrivingReq 
            state = CONST_STATE_STANDSTILL;
%             status = CONST_MSG_NO_DRIVING_REQ;
            
        end
        
    case CONST_STATE_STOPREQ
        steering = false;
        movement = false;
        stopReq = true;
%         status = CONST_STATE_STOPREQ;
        
        vS = tempSetpointVs;
        alpha = statLastAngle;
        
        if ~tempDrivingReq 
            state = CONST_STATE_STANDSTILL;
%             status = CONST_MSG_NO_DRIVING_REQ;
        end
        
%     else
%         state = CONST_STATE_STANDSTILL;
        
end

statLastVelocity = v;
statRotationOnly = rotationOnly;












% % % % % % % %% Kinematic calculations
% % % % % % % if abs(v(1)) < CONST_NULL_P && abs(v(3)) < CONST_NULL_P 
% % % % % % %     tempAlpha = statLastAngle; 
% % % % % % %     tempVs = abs(v(3)) * axesParam.position.x; 
% % % % % % %     
% % % % % % % elseif abs(v(1)) < CONST_NULL_P && abs(v(3)) > 0 
% % % % % % % 
% % % % % % %     if v(3) < 0 
% % % % % % %         tempAlpha = -pi/2;          
% % % % % % %     elseif v(3) > 0 
% % % % % % %         tempAlpha = pi/2; 
% % % % % % %     end
% % % % % % %     tempVs = abs(v(3)) * axesParam.position.x;  
% % % % % % %     
% % % % % % % else
% % % % % % %     tempAlpha = atan(v(3) * axesParam.position.x / v(1)); % in [rad]
% % % % % % %     tempVs = v(1) / cos(tempAlpha); 
% % % % % % % end
% % % % % % % 
% % % % % % % %% decentralized traction wheel
% % % % % % % if abs(tempAlpha) > CONST_NULL_P && axesParam.position.y ~= 0 
% % % % % % %     
% % % % % % %     tempAlphaNew = atan(axesParam.position.x / ( axesParam.position.x / tan(tempAlpha) - axesParam.position.y ) );
% % % % % % %     
% % % % % % %     if axesParam.position.y < 0 
% % % % % % %         if tempAlpha < atan(axesParam.position.x / axesParam.position.y )
% % % % % % %             tempAlphaNew = tempAlphaNew - pi; 
% % % % % % %         end
% % % % % % %     elseif axesParam.position.y > 0 
% % % % % % %         if tempAlpha > atan(axesParam.position.x / axesParam.position.y ) 
% % % % % % %             tempAlphaNew = tempAlphaNew + pi; 
% % % % % % %         end
% % % % % % %     end
% % % % % % %      
% % % % % % %     tempVs = tempVs * sin(tempAlpha) / sin(tempAlphaNew); 
% % % % % % %     tempAlpha = tempAlphaNew; 
% % % % % % % end 
% % % % % % % 
% % % % % % % 
% % % % % % % 
% % % % % % % 
% % % % % % % %% StateMachine :: steer | move | stop
% % % % % % % tempDrivingReq = false; 
% % % % % % % if abs(v(1)) > 0 || abs(v(3)) > 0 
% % % % % % %     tempDrivingReq = true; 
% % % % % % % end
% % % % % % % tempAlphaDiff = abs(tempAlpha - currentAlpha);
% % % % % % % tempAngleValid = 1;%(tempAlpha < invalidRange(1) || tempAlpha > invalidRange(2));
% % % % % % % 
% % % % % % % switch state
% % % % % % %     case CONST_STATE_STANDSTILL
% % % % % % %         steering = false; 
% % % % % % %         movement = false;
% % % % % % %         stopRq = false;
% % % % % % %         
% % % % % % %         vs = 0; 
% % % % % % %         alpha = statLastAngle;
% % % % % % % 
% % % % % % %         if tempDrivingReq %&& tempAlphaDiff < pi/2 && tempAngleValid
% % % % % % %             state = CONST_STATE_WHEELPOSITIONING;
% % % % % % %             
% % % % % % % %         elseif tempDrivingReq && tempAlphaDiff >= pi/2 
% % % % % % % %             state = CONST_STATE_WHEELPOSITIONING_INV;
% % % % % % %         end
% % % % % % % 
% % % % % % %     case CONST_STATE_WHEELPOSITIONING
% % % % % % %         steering = true; 
% % % % % % %         movement = false; 
% % % % % % %         stopRq = false; 
% % % % % % %         
% % % % % % %         vs = 0; 
% % % % % % %         alpha = tempAlpha;
% % % % % % %         statLastAngle = currentAlpha; 
% % % % % % %         
% % % % % % %         if ~tempDrivingReq
% % % % % % %             state = CONST_STATE_STANDSTILL;
% % % % % % %             
% % % % % % %         elseif abs(alpha - currentAlpha) < angularTolerance
% % % % % % %             state = CONST_STATE_MOVEMENT; 
% % % % % % %         end
% % % % % % %         
% % % % % % % %     case CONST_STATE_WHEELPOSITIONING_INV
% % % % % % % %         steering = true; 
% % % % % % % %         movement = false; 
% % % % % % % %         stopRq = false; 
% % % % % % % %         
% % % % % % % %         vs = 0; 
% % % % % % % %         alpha = tempAlpha + pi;
% % % % % % % %         statLastAngle = currentAlpha; 
% % % % % % % %         
% % % % % % % %         if ~tempDrivingReq
% % % % % % % %             state = CONST_STATE_STANDSTILL;
% % % % % % % %         elseif abs(alpha - currentAlpha) < angularTolerance
% % % % % % % %             state = CONST_STATE_MOVEMENT_INV; 
% % % % % % % %         end
% % % % % % %         
% % % % % % %     case CONST_STATE_MOVEMENT
% % % % % % %         steering = false; 
% % % % % % %         movement = true; 
% % % % % % %         stopRq = false;
% % % % % % %         
% % % % % % %         vs = tempVs; 
% % % % % % %         alpha = tempAlpha;
% % % % % % %         statLastAngle = currentAlpha; 
% % % % % % %         
% % % % % % %         if ~tempDrivingReq
% % % % % % %             state = CONST_STATE_STANDSTILL; 
% % % % % % %             
% % % % % % %         elseif (((v(1) < 0 && statLastVelo(1) > 0) && ...
% % % % % % %                  (v(3) > 0 && statLastVelo(3) > 0) ) ... % 2
% % % % % % %                || ...
% % % % % % %                  (v(1) < 0 && statLastVelo(1) > 0) && ...
% % % % % % %                  (v(3) < 0 && statLastVelo(3) < 0) ) ... % 6
% % % % % % %                || ...
% % % % % % %                ( (v(1) > 0 && statLastVelo(1) < 0) && ...
% % % % % % %                  (v(3) > 0 && statLastVelo(3) > 0) ) ... % 7
% % % % % % %                || ...
% % % % % % %                ( (v(1) < 0 && statLastVelo(1) < 0) && ...
% % % % % % %                  (v(3) < 0 && statLastVelo(3) > 0) ) ... % 9
% % % % % % %                || ...
% % % % % % %                ( (v(1) > 0 && statLastVelo(1) < 0) && ...
% % % % % % %                  (v(3) < 0 && statLastVelo(3) < 0) )     % 11
% % % % % % %                  % eventuell auch noch ab gewisser Abweichung einen stop
% % % % % % %                  % fordern 
% % % % % % %           state = CONST_STATE_STOPREQ;
% % % % % % %           vs = statLastVs;
% % % % % % %           alpha = statLastAngle;
% % % % % % %         end
% % % % % % %         
% % % % % % % %       case CONST_STATE_MOVEMENT_INV
% % % % % % % %         steering = false; 
% % % % % % % %         movement = true; 
% % % % % % % %         stopRq = false;
% % % % % % % %         
% % % % % % % %         vs = -tempVs; 
% % % % % % % %         alpha = tempAlpha + pi;
% % % % % % % %         statLastAngle = currentAlpha; 
% % % % % % % %         
% % % % % % % %         if ~tempDrivingReq
% % % % % % % %             state = CONST_STATE_STANDSTILL; 
% % % % % % % %         elseif ( (v(1) < 0 && statLastVelo(1) > 0) && ...
% % % % % % % %                  (v(3) > 0 && statLastVelo(3) > 0) ) ... % 2
% % % % % % % %                || ...
% % % % % % % %                ( (v(1) < 0 && statLastVelo(1) > 0) && ...
% % % % % % % %                  (v(3) < 0 && statLastVelo(3) < 0) ) ... % 6
% % % % % % % %                || ...
% % % % % % % %                ( (v(1) > 0 && statLastVelo(1) < 0) && ...
% % % % % % % %                  (v(3) > 0 && statLastVelo(3) > 0) ) ... % 7
% % % % % % % %                || ...
% % % % % % % %                ( (v(1) < 0 && statLastVelo(1) < 0) && ...
% % % % % % % %                  (v(3) < 0 && statLastVelo(3) > 0) ) ... % 9
% % % % % % % %                || ...
% % % % % % % %                ( (v(1) > 0 && statLastVelo(1) < 0) && ...
% % % % % % % %                  (v(3) < 0 && statLastVelo(3) < 0) )     % 11
% % % % % % % %                  % eventuell auch noch ab gewisser Abweichung einen stop
% % % % % % % %                  % fordern 
% % % % % % % %           state = CONST_STATE_STOPREQ;
% % % % % % % %           vs = statLastVs;
% % % % % % % %           alpha = statLastAngle;
% % % % % % % %         end
% % % % % % %         
% % % % % % %     case CONST_STATE_STOPREQ
% % % % % % %         steering = false; 
% % % % % % %         movement = false; 
% % % % % % %         stopRq = true; 
% % % % % % %         
% % % % % % %         vs = -tempVs;
% % % % % % %         alpha = statLastAngle;
% % % % % % %         statLastAngle = currentAlpha;
% % % % % % % 
% % % % % % %         if ~tempDrivingReq 
% % % % % % %             state = CONST_STATE_STANDSTILL;
% % % % % % %         end
% % % % % % % end
% % % % % % % 
% % % % % % % %% State Machine :: inverse | reverse
% % % % % % % % switch stateInversion
% % % % % % % %     case CONST_INVERSION_NORMAL
% % % % % % % %         if tempAlpha > invalidRange(1) || ... 
% % % % % % % %            tempAlpha < invalidRange(2) 
% % % % % % % %             stateInversion = CONST_INVERSION_INVERT;
% % % % % % % %             state = CONST_STATE_STOPREQ;
% % % % % % % %             alpha = alpha + pi; 
% % % % % % % %             vs = -vs; 
% % % % % % % %         end
% % % % % % % %         
% % % % % % % %         if abs(alpha - currentAlpha) > pi/2 
% % % % % % % %             stateInversion = CONST_INVERSION_INVERT;
% % % % % % % %             state = CONST_STATE_STOPREQ;
% % % % % % % %             alpha = alpha + pi; 
% % % % % % % %             vs = -vs; 
% % % % % % % %         end
% % % % % % % %         
% % % % % % % %     case CONST_INVERSION_INVERT
% % % % % % % %         alpha = alpha + pi; 
% % % % % % % %         vs = -vs; 
% % % % % % % %         
% % % % % % % %         if tempAlpha > invalidRange(1) || ... 
% % % % % % % %            tempAlpha < invalidRange(2) 
% % % % % % % %             stateInversion = CONST_INVERSION_NORMAL;
% % % % % % % %         end
% % % % % % % %             
% % % % % % % %         if abs(alpha - currentAlpha) < pi/4 
% % % % % % % %             stateInversion = CONST_INVERSION_INVERT_2;
% % % % % % % %         end
% % % % % % % %         
% % % % % % % %     case CONST_INVERSION_INVERT_2
% % % % % % % %         alpha = alpha + pi; 
% % % % % % % %         vs = -vs; 
% % % % % % % %         
% % % % % % % %         if tempAlpha > invalidRange(1) || ... 
% % % % % % % %            tempAlpha < invalidRange(2) 
% % % % % % % %             stateInversion = CONST_INVERSION_NORMAL;
% % % % % % % %             state = CONST_STATE_STOPREQ;
% % % % % % % %         end
% % % % % % % %         
% % % % % % % %         if abs(alpha - currentAlpha) > pi/2 
% % % % % % % %             stateInversion = CONST_INVERSION_REVERT;
% % % % % % % %         end
% % % % % % % %         
% % % % % % % %     case CONST_INVERSION_REVERT
% % % % % % % %         if tempAlpha > invalidRange(1) || ... 
% % % % % % % %            tempAlpha < invalidRange(2) 
% % % % % % % %             stateInversion = CONST_INVERSION_INVERT;
% % % % % % % %             state = CONST_STATE_STOPREQ;
% % % % % % % %            alpha = alpha + pi; 
% % % % % % % %            vs = -vs; 
% % % % % % % %         end
% % % % % % % %         
% % % % % % % %         if abs(alpha - currentAlpha) < pi/4 
% % % % % % % %             stateInversion = CONST_INVERSION_NORMAL;
% % % % % % % %             alpha = alpha + pi; 
% % % % % % % %             vs = -vs; 
% % % % % % % %         end
% % % % % % % % end
% % % % % % % statLastVs = vs; 
% % % % % % % statLastVelo = v;
end