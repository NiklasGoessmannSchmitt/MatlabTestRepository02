function [v, maxOmegaError] = LSimoveC_UC_MoveCtrl_NPivot_W2V(n, PivotValues, epsilon, PivotParam)
% (*******************************************************************
% // Description:
% // This function block calculates the forward kinematic of the AGV 
% // based on the velocities and the angles of each wheel relative to 
% // its position in the vehicle.
% // 
% //    1. In the calculation two wheels are compared to calculate an resultig rotational velocity.
% //    2. The average of all partial rotational velocities defines the vehicle rotational velocity
% //    3. After determining the rotational velocity of the vehicle the vehicle velocity in x and y can be determined.
% //
% // Mathematical basis:
%  // Combine two wheels in order to calculate it's rotational velocity
%  // 
%  // vXWheel = vXVehicle - Omega * DistToRootY
%  // vYWheel = vYVehicle + Omega * DistToRootX
%  // 
%  // Special Cases:
%  //    1. DistToRootX = 0
%  //       vXWheel = vXVehicle - Omega * DistToRootY
%  //       vYWheel = vYVehicle
%  //
%  //    2. DistToRootY = 0
%  //       vXWheel = vXVehicle
%  //       vYWheel = vYVehicle + Omega * DistToRootX
%  //       
%  //    3. DistToRootX = 0, DistToRootY = 0
%  //       vXWheel = vXVehicle
%  //       vYWheel = vYVehicle
%  //       
%  //    4. DistToRootY1 = DistToRootY2
%  //        vXWheel1 = vXVehicle - Omega * DistToRootY1
%  //        vXWheel2 = vXVehicle - Omega * DistToRootY1
%  //        --> vXWheel1 = vXWheel2 --> No differential speed calculatable
%  //        
%  //    5. DistToRootX1 = DistToRootX2
%  //        vYWheel1 = vYVehicle + Omega * DistToRootX1
%  //        vYWheel2 = vYVehicle + Omega * DistToRootX1
%  //        --> vYWheel1 = vYWheel2 --> No differential speed calculatable
%  
% // Version
% // 2020-03-17 DG: First Version
% //               
% //
% // Authors
% // DG: Daniel Gauglitz
% //
% //
% // © Alle Rechte vorbehalten/All rights reserved, Siemens AG, 2018
% *******************************************************************)

% % persistent first_cycle
% % persistent statPositionCenter
% % persistent statPositionCenterMap
% % persistent statTravelledDistance
% % persistent setError

% % if isempty(first_cycle) 
% %     first_cycle = true; 
% %     setError = false; 
% %     statPositionCenter = zeros(1,3);
% %     statPositionCenterMap = zeros(1,3);
% %     statTravelledDistance = 0;
% % end

tempWheelVelocitiesVector = zeros(n,3);
tempOmegaError = false; 
tempVXVehicle = zeros(1,n); 
tempVYVehicle = zeros(1,n); 


%%

% % if first_cycle 
% %    first_cycle = false;
% %    
% %    setError = false;
% %    
% %    statPositionCenter(1) = 0;
% %    statPositionCenter(2) = 0;
% %    statPositionCenter(3) = 0;
% %    
% %    statPositionCenterMap(1) = 0;
% %    statPositionCenterMap(2) = 0;
% %    statPositionCenterMap(3) = 0;
% %    
% %    statTravelledDistance = 0;
% % end

% % if Reset 
% %    first_cycle = false;
% %    
% %    setError = false;
% %    
% %    statPositionCenter = ResetPosition;
% %    statPositionCenterMap = ResetPosition; 
% %    statTravelledDistance = DistanceValue;
% % end

%% Calculate Wheel velocities in vehicle coordinates
for tempWheelCounter = 1:n
   tempWheelVelocitiesVector(tempWheelCounter,1) = PivotValues(tempWheelCounter).velocity;
   tempWheelVelocitiesVector(tempWheelCounter,2) = 0;
   tempWheelVelocitiesVector(tempWheelCounter,3) = 0;
   
   tempWheelVelocitiesInVehicle(tempWheelCounter,:) = frameRotate(tempWheelVelocitiesVector(tempWheelCounter,:), PivotValues(tempWheelCounter).angularPosition);
end

 %% Calculate Vehicle odometry

    %____________________________________________________________________________________________________________________________________
        % Calc Omega using X-Velocity of two different wheels 
    tempCntOmega = 0;
    for tempWheelCounter = 1:n
       for tempWheelCounter_1 = 1:n 
          % Check if maximum indes is exceeded
          if tempCntOmega > 100 
             break
          end
         
         % Check the conditions under which the calculation is not possible (see long comment above)
          if tempWheelCounter_1 <= tempWheelCounter || ...
...%              PivotParam(tempWheelCounter).position.y == 0 || ...
...%              PivotParam(tempWheelCounter_1).position.y == 0 || ...
             PivotParam(tempWheelCounter).position.y == PivotParam(tempWheelCounter_1).position.y
          else          
              % Calculate partial omega for the two selected wheels
              tempOmegaCalc(tempCntOmega+1) = (tempWheelVelocitiesInVehicle(tempWheelCounter_1,1) - tempWheelVelocitiesInVehicle(tempWheelCounter,1)) / ...
                    (PivotParam(tempWheelCounter).position.y - PivotParam(tempWheelCounter_1).position.y);
              tempCntOmega = tempCntOmega + 1;
          end
       end
    end
    
    %____________________________________________________________________________________________________________________________________
    % Calc Omega using Y-Velocity using two different wheels
    for tempWheelCounter = 1:n 
       for tempWheelCounter_1 = 1:n 
          % Check if maximum indes is exceeded
          if tempCntOmega > 100 
             break
          end
          
          % Check the conditions under which the calculation is not possible (see long comment above)
          if tempWheelCounter_1 <= tempWheelCounter || ...
...%              PivotParam(tempWheelCounter).position.x == 0 || ...
...%              PivotParam(tempWheelCounter_1).position.x == 0 || ...
             PivotParam(tempWheelCounter).position.x == PivotParam(tempWheelCounter_1).position.x
          
%              CONTINUE; 
          else
             % Calculate partial omega for the two selected wheels
              tempOmegaCalc(tempCntOmega+1) = (tempWheelVelocitiesInVehicle(tempWheelCounter_1,2) - tempWheelVelocitiesInVehicle(tempWheelCounter,2)) / ...
                        (PivotParam(tempWheelCounter_1).position.x - PivotParam(tempWheelCounter).position.x);
              tempCntOmega = tempCntOmega + 1;
          end
       end
    end
    
    %____________________________________________________________________________________________________________________________________
    % Calculate Vehicle Omega from partial Omega-calculations
    tempVelocityCenter.a = 0;
    for tempWheelCounter = 1:(tempCntOmega)
       tempVelocityCenter.a = tempVelocityCenter.a + tempOmegaCalc(tempWheelCounter);
    end
    tempVelocityCenter.a = tempVelocityCenter.a / tempCntOmega;
    
 
 %% Check for Omega error
 for tempWheelCounter = 1:(tempCntOmega)
     maxOmegaError = max(abs(tempVelocityCenter.a) - abs(tempOmegaCalc(tempWheelCounter)));
    if abs( abs(tempVelocityCenter.a) - abs(tempOmegaCalc(tempWheelCounter))) > epsilon 
       tempOmegaError = true;
    end
 end

 
 %% Calculate X und Y velocity of AGV wheel-depending
%    // Basis:
%    // vXWheel = vXVehicle - Omega * DistToRootY
%    // vYWheel = vYVehicle + Omega * DistToRootX
%    // tempOmegaMediums is Omega of vehicle

   % X-Velocity Center
   for tempWheelCounter = 1:n 
      tempVXVehicle(tempWheelCounter) = tempWheelVelocitiesInVehicle(tempWheelCounter,1) + ...
                tempVelocityCenter.a * PivotParam(tempWheelCounter).position.y;
      
      % Y-Velocity Center
      tempVYVehicle(tempWheelCounter) = tempWheelVelocitiesInVehicle(tempWheelCounter,2) - ...
                tempVelocityCenter.a * PivotParam(tempWheelCounter).position.x;
   end

%% Calculate X and Y velocity of AGV
   tempVelocityCenter.x = 0;
   tempVelocityCenter.y = 0;
   for tempWheelCounter = 1:n 
      tempVelocityCenter.x = tempVelocityCenter.x + tempVXVehicle(tempWheelCounter);
      tempVelocityCenter.y = tempVelocityCenter.y + tempVYVehicle(tempWheelCounter);
   end
   v(1) = tempVelocityCenter.x / n;
   v(2) = tempVelocityCenter.y / n;
   v(3) = tempVelocityCenter.a; 