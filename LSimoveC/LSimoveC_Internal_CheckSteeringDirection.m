function [posSteeringAllowed, negSteeringAllowed] = LSimoveC_Internal_CheckSteeringDirection (actualAngle, targetAngle, forbiddenAngles)
% REGION BLOCK INFO HEADER
%     //===============================================================================
%     // Siemens AG / (c)Copyright 2018
%     //-------------------------------------------------------------------------------
%     // Title:            LSimoveC_Internal_CheckSteeringDirection
%     // Comment/Function: 
%     // Library/Family:   SIMOVE Carrier Control
%     // Author:           Siemens Digital Industries
%     // Tested with:      OpenController V2 SoftPLC V2.7
%     // Engineering:      TIA Portal V15.1
%     //-------------------------------------------------------------------------------
%     // Change log table:
%     // Version  | Date       | Expert in charge | Changes applied
%     //----------|------------|------------------|------------------------------------
%     // 01.00.00 | 13.05.2020 | Siemens Industry | First Version
%     // 03.00.00 | 06.07.2020 | Siemens Industry | Released Version V3
%     //===============================================================================
% END_REGION
% REGION DESCRIPTION
% (*    
% (/*
% # Steering direction check
% In this function checks the validity of steering directions depending on the steering deviation with respect to forbidden angles.
%  
% * Input: Forbidden angles pairs i.e. calculated based on the vehicle setpoint velocity and the wheel's position in the AGV.
% 
% * Output: 
% 
% Positive steering allowed
% 
% Negative steering allowed
% */)
% *)
% END_REGION DESCRIPTION

act = actualAngle;
target = targetAngle;

%  Init allowance of steering directions
posSteeringValid = true;
negSteeringValid = true;

%% First forbidden angle
   % Read forbidden angles
   higher = forbiddenAngles(1).higher;
   lower = forbiddenAngles(1).lower;
   
   if (higher - lower) > 0 
      
      if act == target 
         ;
      elseif act > target
         % Forbidden angle area between actual and target angle
         if act >= higher && target < lower 
            negSteeringValid = false;
         end
         % Forbidden angle area smaller than target angle
         if act >= higher && target > higher 
            posSteeringValid = false;
         end
         % Forbidden angle area bigger than actual angle
         if act <= lower && target < lower 
            posSteeringValid = false;
         end
      elseif act < target 
         % Forbidden angle area between actual and aarget angle
         if act > higher && target > higher 
            negSteeringValid = false;
         end
         % Forbidden angle area smaller than target angle
         if act < lower && target > higher 
            posSteeringValid = false;
         end
         % Forbidden angle area bigger than actual angle
         if act < lower && target < lower 
            negSteeringValid = false;
         end
      end
   end


%% Second forbidden angle
   % Read forbidden angles
   higher = forbiddenAngles(2).higher;
   lower = forbiddenAngles(2).lower;
   
   if (higher - lower) > 0 
      
      if act == target 
         ;
      elseif act > target 
         % Forbidden angle area between actual and target angle
         if act >= higher && target < lower 
            negSteeringValid = false;
         end
         % Forbidden angle area smaller than target angle
         if act >= higher && target > higher 
            posSteeringValid = false;
         end
         % Forbidden angle area bigger than actual angle
         if act <= lower && target < lower 
            posSteeringValid = false;
         end
      elseif act < target 
         % Forbidden angle area between actual and aarget angle
         if act > higher && target > higher 
            negSteeringValid = false;
         end
         % Forbidden angle area smaller than target angle
         if act < lower && target > higher 
            posSteeringValid = false;
         end
         % Forbidden angle area bigger than actual angle
         if act < lower && target < lower 
            negSteeringValid = false;
         end
      end
   end

%% Actual angle is within forbidden angle zone --> take shortest way out
if act < higher && act > lower 
    if (higher - act) >= (act - lower) 
        posSteeringValid = false;
        negSteeringValid = true;
    else
        posSteeringValid = true;
        negSteeringValid = false;
    end
end

%% Write outputs
posSteeringAllowed = posSteeringValid;
negSteeringAllowed = negSteeringValid;