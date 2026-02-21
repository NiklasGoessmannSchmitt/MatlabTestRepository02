close all
clear all 
clc

%%
addpath '../BezierTransformation'
addpath '../LSimoveC'

%% Simulation Settings
enablePlots = true; 
enablePredictiveLaneControl = false;
samplingTime = 1e-3; % in [s]

%% Generate Path 
kurveRadius = 3; 
 [xq, yq, dq, endIDindx, xAGV, yAGV, aAGV, b] = getTrack('sCurve',kurveRadius); 
%[xq, yq, dq, endIDindx, xAGV, yAGV, aAGV, b] = getTrackExternal('basic'); 

% JONAS: distance2curve ersetzen durch bezier DONE 
% JONAS: push target mit bisschen abstand starten, mean(dq) mal punkte DONE
% nicht bei agv starten
% TO DO: export import SICK Scanner mit mehreren feldern DONE
% TO DO: bezier bei push to target?? länge bezier elemente frage..

% Feldumschaltung mit x unterschiedlichen Feldern
% Feldauslegung (erstmal yolo, danach formel)
% so tun als ob ich mit feld fahre -> ich krieg feedback wenns knallt 
% optimierer auf geschwindigkeit
useExternalVTSData = false; 
if useExternalVTSData 
    externalVts = [2,2,pi/2]; 
end
% sCurve
% BMF

%% AGV setup
[pivotParam, agvType, motor, n, agvMeasures, pivotValues, sensor, name, control, velocity] = AGVconfig('M240', samplingTime); 
% help AGVconfig - to see what AGV models are available

maxSimTime = 250; % in [s]
simSteps = maxSimTime / samplingTime; 

navigationMode = 0; 

%% Initialize some values
% g -- general
% a -- agv
g = initGeneralValues(xAGV, yAGV, aAGV, simSteps);
a = initAgvValues(g, n, agvType, motor, agvMeasures, sensor, pivotParam, pivotValues);

agvCenter = zeros(simSteps,2); 
positioning = false; 
positioningIndex = 0; 
targetReached = false; 
errorBit = false; 
errorIndex = 0;

j = 0; 

alternateMapPosOld = zeros(3,1);
target = zeros(3,1); 

%% RUN simulation 
for i = 1:g.simSteps
% tic   
if strcmp(sensor.type, 'ANS')   
    %% ANS
    if ~useExternalVTSData      
        if sensor.ans.targetXValue > 0 
%             if i == 1 
                statAnsTargetDistance = sensor.ans.targetXValue;
%             elseif finalTargetLength > sensor.ans.targetXValue
%                 statAnsTargetDistance = statAnsTargetDistance + ( sensor.ans.targetXValue - target(1) )*0.1; 
%             end
        else 
            statAnsTargetDistance = sensor.ans.targetDistance; 
        end 
%         tic
        [target, finalTargetLength, targetLength, carrotGlobal, tdPoint] = ansEmulation(xq,yq,dq,endIDindx,endIDindx,statAnsTargetDistance,g,b);
%         toc
    else
        target = externalVts; 
        finalTargetLength = sqrt( externalVts(1)^2 + externalVts(2)^2 );
        targetLength = finalTargetLength; 
        carrotGlobal(1) = g.curMapPos(1) + target(1)*cos(g.curMapPos(3)); 
        carrotGlobal(2) = g.curMapPos(2) + target(2)*sin(g.curMapPos(3)); 
        carrotGlobal(3) = g.curMapPos(3) + target(3); 
    end
    
    %% Positioning
    if targetLength < sensor.ans.targetDistance && ~positioning
        disp('Positioning onto target!')
        positioning = true; 
        positioningIndex = i; 
    end
    
    %% Check for target radius
    targetRadius = 0.01; % [m]
    if sqrt(target(1)^2+target(2)^2) < targetRadius 
%         if target(1) <= 0.0 %% half circle
            if ~targetReached 
                disp('Target Reached!')
                fprintf(['X deviation: ', num2str(target(1)*1000), ' mm \n',...
                         'Y deviation: ', num2str(target(2)*1000), ' mm \n',...
                         'A deviation: ', num2str(target(3)*180/pi), ' ° \n'])
                targetReached = true; 
                targetReachedIndx = i; 
            end
            target = target .* 0;
            if i > (targetReachedIndx + 0.5/samplingTime) % wait for 0.5 seconds
                break;
            end
%         end 
    else 
        if i > 25 % Target FIR 
            target(1) = sum( [g.trackDeviationBuffer(1,(i-19:i-1)), target(1)] ) / 20; 
            target(2) = sum( [g.trackDeviationBuffer(2,(i-19:i-1)), target(2)] ) / 20; 
            target(3) = sum( [g.trackDeviationBuffer(3,(i-19:i-1)), target(3)] ) / 20; 
        end
    end
    
    %% Position Controller
    g.trackDeviationBuffer(:,i) = target; 
    
    if sensor.ans.targetXValue > 0 %&& i == -5 
        %% Track Control
        target(2) = -target(2); 
        target(3) = -target(3); 
        
        propPartY(i) = (0-target(2))*control.y.pTrack;
        intePartY(i) = (0-target(2))*control.y.iTrack + a.intePartoldY;
        if i > 1
            diffPartY(i) = (g.trackDeviationBuffer(2,i-1) - target(2))*control.y.dTrack;
        else
            diffPartY(i) = 0; 
        end
        
        propPartA(i) = (0-target(3))*control.a.pTrack;
        intePartA(i) = (0-target(3))*control.a.iTrack + a.intePartoldA;
        if i > 1
            diffPartA(i) = (g.trackDeviationBuffer(3,i-1) - target(3))*control.a.dTrack;
        else
            diffPartA(i) = 0; 
        end
        
        velocityA(i) = propPartY(i) + intePartY(i) + diffPartY(i)... 
                     + propPartA(i) + intePartA(i) + diffPartA(i);
        
        a.intePartoldY = intePartY(i);
        a.intePartoldA = intePartA(i); 
            
        if finalTargetLength > velocity.trackSpeedTarget
            finalTargetLength = velocity.trackSpeedTarget; 
        end
        vSet = [finalTargetLength; 0; velocityA(i)];

                
    elseif strcmp(agvType, 'Tricycle') || strcmp(agvType, 'Diff') ...
            || strcmp(name, 'BMF')
        
        k_phi_only_range = 0.0;%0.2;
        steer_limit_range = sensor.ans.targetDistance*.9;
        v = XA_Pilot(target, targetLength, sensor.ans.targetDistance, a.betaSetpoints, ...
            k_phi_only_range, steer_limit_range, velocity.trackSpeedTarget, finalTargetLength);
        vSet = v'; 

    elseif strcmp(agvType, 'Mecanum') || strcmp(agvType, 'PivotActive') || strcmp(agvType, 'PivotPassive')

        targetOrientation = 0;%pi/2; 
        v = XYA_Pilot(target,g.curMapPos,targetOrientation,navigationMode,...
            velocity.trackSpeedTarget,targetLength,finalTargetLength);
        vSet = v;

    elseif i == 1 
        disp('No pilot for such an AGV!')
    end  

elseif strcmp(sensor.type, 'Y') 
        
    if strcmp(a.type, 'SteeredDiff_SteerMaster') % seperate case because track sensor is mounted on rotating pivot
        %% Y Sensor 
        steeringPivotPos(1) = g.curMapPos(1) + cos(g.curMapPos(3)) * a.pivotParam(1).position.x; 
        steeringPivotPos(2) = g.curMapPos(2) + sin(g.curMapPos(3)) * a.pivotParam(1).position.x; 
        steeringPivotPos(3) = g.curMapPos(3) + a.pivotValues(1).angularPosition; 
        [trackDeviation, sensorFrame, xi, yi] = trackSensor(xq, yq, steeringPivotPos, a.sensor);
        if ~isnumeric(trackDeviation.y) 
            break;
        end
        g.trackDeviationBuffer(2,i) = trackDeviation.y; 
        
        % SETPOINT
        a.betaSetpoints(1) = a.oldBeta(1) + 0.002 * -trackDeviation.y; 
        
        omegaMaxRadS = 2 * 2.11865346 / 180 * pi; % Geschwindigkeit berechnet von Wolfgang Umhau
        maxAccelerationWheel = 1;
        kp = 1; 
        setpointSteeringVelocity = LSimoveC_UC_Kin_SteeringControl...
            (1, a.betaSetpoints(1), a.oldBeta(1), false, omegaMaxRadS, maxAccelerationWheel, kp, samplingTime);

        steeringVelocity = setpointSteeringVelocity; % tf = 1
        a.pivotValues(1).angularPosition = a.oldBeta(1) + steeringVelocity * samplingTime;
        a.oldBeta(1) = a.pivotValues(1).angularPosition; 
        
        vs = 0.1;
        vSet = LSimoveC_UC_MoveCtrl_Tricycle_W2V(vs, a.pivotValues(1).angularPosition, a.pivotParam(1)); 
        
        a.vsCurrentBuffer(i) = vs; 
        a.betaCurrentBuffer(i) = a.pivotValues(1).angularPosition; 
        a.betaSetpointBuffer(i) = a.betaSetpoints;
        a.steeringVelocitySetpointBuffer(i) = setpointSteeringVelocity;
        a.steeringVelocityBuffer(i) = steeringVelocity; 
        
    else    
        %% Y Sensor - default calculation
        [trackDeviation, sensorFrame, xi, yi] = trackSensor(xq, yq, g.curMapPos, a.sensor);
        if ~isnumeric(trackDeviation.y) 
            break;
        end
        g.trackDeviationBuffer(2,i) = trackDeviation.y; 
        %% Track Control
        if g.curMapPos(1) < (0-a.sensor.pos(1))
            propPart(i) = (0-trackDeviation.y)*control.y.pTrack;
            intePart(i) = (0-trackDeviation.y)*control.y.iTrack + a.intePartoldY;
            if i > 1
                diffPart(i) = (g.trackDeviationBuffer(2,i-1) - trackDeviation.y)*control.y.dTrack;
            else
                diffPart(i) = 0; 
            end
            velocityA(i) = propPart(i) + intePart(i) + diffPart(i);
            a.intePartoldY = intePart(i);
        else
            pXs = g.curMapPos(1) + velocity.trackSpeed * samplingTime * cos(g.curMapPos(3));
            pYs = g.curMapPos(2) + velocity.trackSpeed * samplingTime * sin(g.curMapPos(3));   
            [~, angleOverall, e] = findNearestPointNewton(pXs, pYs, g.curMapPos(1), g.curMapPos(2), g.curMapPos(3), [0 kurveRadius kurveRadius], [0 0 kurveRadius], 1e-3);
            velocityA(i) = (angleOverall - g.curMapPos(3)) / samplingTime;
            test = velocityA(i); 
            velocityA(i) = 0;
        end
        vSet = [velocity.trackSpeed; 0; velocityA(i)];
    end
    
elseif strcmp(sensor.type, 'YA')   
     
    %% Sensor 
    [trackDeviation, sensorFrame, xi, yi] = trackSensor(xq, yq, g.curMapPos, a.sensor);
    if ~isnumeric(trackDeviation.y) 
        disp('No valid track values!')
        break;
    end
    g.trackDeviationBuffer(2,i) = trackDeviation.y;
    g.trackDeviationBuffer(3,i) = trackDeviation.a;
    
    %% Track Control
    
    if strcmp(agvType, 'Diff') || strcmp(agvType, 'Tricycle')
        
        propPartY(i) = (0-trackDeviation.y)*control.y.pTrack;
        intePartY(i) = (0-trackDeviation.y)*control.y.iTrack + a.intePartoldY;
        if i > 1
            diffPartY(i) = (g.trackDeviationBuffer(2,i-1) - trackDeviation.y)*control.y.dTrack;
        else
            diffPartY(i) = 0; 
        end
        
        propPartA(i) = (0-trackDeviation.a)*control.a.pTrack;
        intePartA(i) = (0-trackDeviation.a)*control.a.iTrack + a.intePartoldA;
        if i > 1
            diffPartA(i) = (g.trackDeviationBuffer(3,i-1) - trackDeviation.a)*control.a.dTrack;
        else
            diffPartA(i) = 0; 
        end
        
        velocityA(i) = propPartY(i) + intePartY(i) + diffPartY(i)...
                     + propPartA(i) + intePartA(i) + diffPartA(i);
        a.intePartoldY = intePartY(i);
        a.intePartoldA = intePartA(i);
        
        vSet = [velocity.trackSpeed; 0; velocityA(i)];
        
    else       
        orientationOnTrack = 0; % in [deg]

        tempVXStar = velocity.trackSpeed * cos(orientationOnTrack);
        tempVYStar = velocity.trackSpeed * sin(orientationOnTrack);

        tempVXSnake = - trackDeviation.y * cos(pi/2 - trackDeviation.a) * control.y.pTrack;
        tempVYSnake = - trackDeviation.y * sin(pi/2 - trackDeviation.a) * control.y.pTrack;
        tempVASnake = - (orientationOnTrack + trackDeviation.a) *  control.a.pTrack;

        vSet(1) = tempVXStar + tempVXSnake; 
        vSet(2) = tempVYStar + tempVYSnake; 
        vSet(3) = 	           tempVASnake; 
    
% % %     %% rot
% % %     tempvSet(1) = cos(trackDeviation.a) * vSet(1) + sin(trackDeviation.a) * vSet(2); 
% % %     tempvSet(2) = cos(trackDeviation.a) * vSet(2) - sin(trackDeviation.a) * vSet(1); 
% % %     
% % %     vSet = [tempvSet(1); tempvSet(2); vSet(3)]; 
    end 
end

%% Velocity Ramping
[vRamped, ~, ~, accBit, decBit] = LSimoveC_UC_MoveCtrl_Ramp ...
    (0, (~a.statSteering || a.statMovement), 1, vSet, velocity.vMax, velocity.acc, velocity.dec, samplingTime);

g.vRampedBuffer(:,i) = vRamped; 
g.vSetBuffer(:,i) = vSet;

%% Kinematics
if strcmp(agvType, 'Diff')
    [vl, vr] = invKinDiffDrive(vRamped,a.pivotParam);
    
    [vl, a.zl] = myFilter(motor(1,:),motor(2,:),vl,a.zl); % transfer function
    [vr, a.zr] = myFilter(motor(1,:),motor(2,:),vr,a.zr); % transfer function   
    a.vlBuffer(i) = vl;
    a.vrBuffer(i) = vr; 
    
    vCurrent = fwdKinDiffDrive(vl,vr,a.pivotParam); 
    g.vCurrent(:,i) = vCurrent; 

elseif strcmp(agvType, 'SteeredDiff_SteerMaster')
    [vl, vr] = invKinDiffDrive(vRamped,a.pivotParam(2));
    
    a.vlBuffer(i) = vl;
    a.vrBuffer(i) = vr; 
    
    vCurrent = fwdKinDiffDrive(vl,vr,a.pivotParam(2)); 
    g.vCurrent(:,i) = vCurrent; 
    
elseif strcmp(agvType, 'SteeredDiff_DiffMaster')
    [vl, vr] = invKinDiffDrive(vRamped,a.pivotParam(2));
    
    a.vlBuffer(i) = vl;
    a.vrBuffer(i) = vr; 
    
    vCurrent = fwdKinDiffDrive(vl,vr,a.pivotParam(2)); 
    g.vCurrent(:,i) = vCurrent; 
    
    
    angularTolerance = pi; % in [rad]
    [vs, a.betaSetpoints(1), steering, movement] = LSimoveC_UC_Kin_Tricycle_V2W(vCurrent, a.pivotParam(1), a.oldBeta(1), angularTolerance, [pi/2*1.05, -pi/2*1.05]); 
    
    % SETPOINT
    releaseSteering = 1;
    steerShortestWay = false;
    omegaMaxRadS = 1.5; 
    maxAccelerationWheel = 1.0;
    kp = 0.002; 
    setpointSteeringVelocity = LSimoveC_UC_Kin_SteeringControl...
        (releaseSteering, a.betaSetpoints(1), a.oldBeta(1), steerShortestWay, omegaMaxRadS, maxAccelerationWheel, kp, samplingTime);

%     [steeringVelocity, a.z] = myFilter(motor(1,:),motor(2,:),setpointSteeringVelocity,a.z); % transfer function
    steeringVelocity = setpointSteeringVelocity; % tf = 1
    beta = a.oldBeta(1) + steeringVelocity * samplingTime;
    a.pivotValues(1).angularPosition = beta;
    
    a.oldBeta(1) = beta;
    a.vsCurrentBuffer(i) = vs; 
    a.betaCurrentBuffer(i) = beta; 
    a.betaSetpointBuffer(i) = a.betaSetpoints;
    a.steeringVelocitySetpointBuffer(i) = setpointSteeringVelocity;
    a.steeringVelocityBuffer(i) = steeringVelocity; 
%     vCurrent = LSimoveC_UC_MoveCtrl_Tricycle_W2V(vs, beta, a.pivotParam);    
    
elseif strcmp(agvType, 'Tricycle')   
    
    angularTolerance = 0.03; % in [rad]
    [vs, a.betaSetpoints, steering, movement] = LSimoveC_UC_Kin_Tricycle_V2W(vRamped, a.pivotParam, a.oldBeta, angularTolerance, [pi/2*1.05, -pi/2*1.05]); 

    vsmin = 0.01;
    if abs(vs) < vsmin && vs ~= 0.0
        vmin = LSimoveC_UC_MoveCtrl_Tricycle_W2V(vsmin, beta, a.pivotParam);
        [vs, a.betaSetpoints, steering, movement] = LSimoveC_UC_Kin_Tricycle_V2W(vmin, a.pivotParam, a.oldBeta, angularTolerance, [pi/2*1.05, -pi/2*1.05]); 
    end
    
    % SETPOINT
    releaseSteering = 1;
    steerShortestWay = false;
    setpointSteeringVelocity = LSimoveC_UC_Kin_SteeringControl...
        (releaseSteering, a.betaSetpoints, a.oldBeta, steerShortestWay, ...
        control.pivot.omegaMaxRadS, ...
        control.pivot.maxAccelerationWheel, ...
        control.pivot.kp, samplingTime);

%     [steeringVelocity, a.z] = myFilter(motor(1,:),motor(2,:),setpointSteeringVelocity,a.z); % transfer function
    steeringVelocity = setpointSteeringVelocity; % tf = 1
    beta = a.oldBeta + steeringVelocity * samplingTime;
    a.pivotValues.angularPosition = beta;
    
    vCurrent = LSimoveC_UC_MoveCtrl_Tricycle_W2V(vs, beta, a.pivotParam); 
    
    g.vCurrent = vCurrent';
    a.statSteering = steering; 
    a.statMovement = movement;
    a.oldBeta = beta;
    
    a.vsCurrentBuffer(i) = vs; 
    a.betaCurrentBuffer(i) = beta; 
    a.betaSetpointBuffer(i) = a.betaSetpoints;
    a.steeringVelocitySetpointBuffer(i) = setpointSteeringVelocity;
    a.steeringVelocityBuffer(i) = steeringVelocity; 
    a.steeringBuffer(i) = steering;
    a.movementBuffer(i) = movement; 
    
elseif strcmp(agvType, 'TricycleRear')   
        
    [vs, a.betaSetpoints, steering, movement] = LSimoveC_UC_Kin_Tricycle_V2W(vRamped, a.pivotParam(1), a.oldBeta(1), control.pivot.angularTolerance, [pi/2*1.05, -pi/2*1.05]); 
   
    if abs(vs) > velocity.vsmax
        vRamped = vRamped .* (velocity.vsmax / vs);
        [vs, a.betaSetpoints, steering, movement] = LSimoveC_UC_Kin_Tricycle_V2W(vRamped, a.pivotParam(1), a.oldBeta(1), control.pivot.angularTolerance, [pi/2*1.05, -pi/2*1.05]); 
    end
    
    % SETPOINT
    releaseSteering = 1;
    steerShortestWay = false;
    setpointSteeringVelocity = LSimoveC_UC_Kin_SteeringControl...
        (releaseSteering, a.betaSetpoints(1), a.oldBeta(1), steerShortestWay, control.pivot.omegaMaxRadS, control.pivot.maxAccelerationWheel, control.pivot.kp, samplingTime);

%     [steeringVelocity, a.z] = myFilter(motor(1,:),motor(2,:),setpointSteeringVelocity,a.z); % transfer function
    steeringVelocity = setpointSteeringVelocity; % tf = 1
    beta = a.oldBeta(1) + steeringVelocity * samplingTime;
    a.pivotValues(1).angularPosition = beta;
    
    vCurrent = LSimoveC_UC_MoveCtrl_Tricycle_W2V(vs, beta(1), a.pivotParam(1)); 
    [vl,vr] = invKinDiffDrive(vCurrent, a.pivotParam(2));
    vCurrent2 = fwdKinDiffDrive(vl, vr, a.pivotParam(2)); 
    
    a.statSteering = steering; 
    a.statMovement = movement;
    a.oldBeta = beta;
    
    a.vsCurrentBuffer(i) = vs; 
    a.betaCurrentBuffer(i) = beta; 
    a.betaSetpointBuffer(i) = a.betaSetpoints;
    a.steeringVelocitySetpointBuffer(i) = setpointSteeringVelocity;
    a.steeringVelocityBuffer(i) = steeringVelocity; 
    a.steeringBuffer(i) = steering;
    a.movementBuffer(i) = movement; 
    
elseif strcmp(agvType, 'PivotActive')
    
    rampingFactorSteering = 1; 
    omegaMax = 1000;
    iTolerance = 4e-3;
    iStandstill = true; 
    
%     [vs, a.betaSetpoints] = LSimoveC_UC_MoveCtrl_NPivot_V2W(vRamped, a.pivotValues, a.pivotParam, n, omegaMax, rampingFactorSteering, iTolerance, 0.5, iStandstill, 0);
    [vs, a.betaSetpoints] = LSimoveC_UC_MoveCtrl_NPivot_V2W([0 0 0.1], a.pivotValues, a.pivotParam, n, omegaMax, rampingFactorSteering, iTolerance, 0.5, iStandstill, 0);
    a.betaSetpointBuffer(:,i) = a.betaSetpoints; 
    
    SteerShortestWay = false;
    Release = true; 
    iKp = 1; 
    iOmegaMaxRadS = 1.4;%5; % [rad/s]
    wheelAcceleration = 9; % [rad/s²]
    [setpointSteeringVelocity, rampingFactorSteering] = LSimoveC_UC_MoveCtrl_NP_SteeringControl(n, a.betaSetpoints, SteerShortestWay, Release, iKp, a.pivotValues, iOmegaMaxRadS, wheelAcceleration, samplingTime);
    
    for p = 1:n
        [steeringVelocity(p), a.z(p,:)] = myFilter(motor(1,:),motor(2,:),setpointSteeringVelocity(p),a.z(p,:));
        a.pivotValues(p).angularPosition = a.pivotValues(p).angularPosition + steeringVelocity(p) * samplingTime;
        
        % myFiler -- tf(1)
        a.pivotValues(p).velocity = vs(p);
        
        % buffers
        a.oldBeta(p) = a.pivotValues(p).angularPosition; 
        a.betaCurrentBuffer(p,i) = a.oldBeta(p);
        a.steeringVelocityBuffer(p:i) = steeringVelocity(p);
        a.vsSetpointBuffer(p,i) = vs(p);
    end
    
    epsilon = 0.15;
    v = LSimoveC_UC_MoveCtrl_NPivot_W2V(n, a.pivotValues, epsilon, a.pivotParam);
    vCurrent = v; 
    %% Safety Tests
    vMod = LSimoveC_UC_MoveCtrl_NPivot_W2V_safetyMod(n, a.pivotValues, epsilon, a.pivotParam); 
    
    if mean(abs(v-vMod)) > 0 
        ; 
    end
%     disp('')
    
    
elseif strcmp(agvType, 'PivotPassive')
    
    rampingFactorSteering = 1; 
    omegaMax = 1000;
    iTolerance = 4e-3;
    iStandstill = false; 
    
    [vs, a.betaSetpoints] = LSimoveC_UC_MoveCtrl_NPivot_V2W(vRamped, a.pivotValues, a.pivotParam, n, omegaMax, rampingFactorSteering, iTolerance, 0.5, iStandstill, 0);
    a.betaSetpointBuffer(:,i) = a.betaSetpoints; 
    
    minvs = 10e-3; 
    if min(abs(vs)) >= minvs 
        % nothing
    elseif min(abs(vs)) > 0.0  
        vs = vs*minvs/min(abs(vs));  
    else 
        vs = zeros(1,a.n); 
    end 
    
    a.vsCurrentBuffer(:,i) = vs; 
    
    SteerShortestWay = false;
    Release = true; 
    iKp = 1; 
    iOmegaMaxRadS = 1.4; % [rad/s]
    wheelAcceleration = 9; % [rad/s²]
    [setpointSteeringVelocity, rampingFactorSteering] = LSimoveC_UC_MoveCtrl_NP_SteeringControl(n, a.betaSetpoints, SteerShortestWay, Release, iKp, a.pivotValues, iOmegaMaxRadS, wheelAcceleration, samplingTime);
    
    for p = 1:n    
%         if p == 2 
%             if i == 1 
%                 [vl(p), vr(p)] = LSimoveC_UC_MoveCtrl_Tricycle_Passive_W2M(vs(p),... 
%                     setpointSteeringVelocity(p),...
%                     a.pivotValues(p).angularPosition,...
%                     0, ...
%                     a.pivotParam(p));
%             else 
%                 [vl(p), vr(p)] = LSimoveC_UC_MoveCtrl_Tricycle_Passive_W2M(vs(p), ...
%                     setpointSteeringVelocity(p),...
%                     0,... a.pivotValues(p).angularPosition, ...
%                     g.vCurrentBuffer(3,i-1), ...
%                     a.pivotParam(p));
%             end
%             %% radius determination 
%             if vl(p) ~= vr(p)
%                 radius(i) = a.pivotParam(p).ringmount / 2.0 .* (vl(p) + vr(p)) / (vl(p) - vr(p));
%             else 
%                 radius(i) = pi; 
%             end
%         else           
            [vl(p), vr(p)] = invKinDiffDrive([vs(p),0,setpointSteeringVelocity(p)],a.pivotParam(p));
%         end
    end

    for p = 1:n 
        vCurrent = fwdKinDiffDrive(vl(p),vr(p),a.pivotParam(p)); 
        steeringVelocity(p) = vCurrent(3); 
        vs(p) = vCurrent(1); 
        
%         [steeringVelocity(p), a.z(p,:)] = myFilter(motor(1,:),motor(2,:),setpointSteeringVelocity(p),a.z(p,:));
        a.pivotValues(p).angularPosition = a.pivotValues(p).angularPosition + steeringVelocity(p) * samplingTime;
        if isnan(a.pivotValues(p).angularPosition) 
            disp('error!!')
        end
        a.pivotValues(p).velocity = vs(p);
        
        % buffers
        a.oldBeta(p) = a.pivotValues(p).angularPosition; 
        a.betaCurrentBuffer(p,i) = a.oldBeta(p);
        a.SteeringVelocityBuffer(p:i) = steeringVelocity(p);
%         a.vlBuffer(p,i) = vl(p);
%         a.vrBuffer(p,i) = vr(p); 
    end
    a.vlBuffer(:,i) = vl;
    a.vrBuffer(:,i) = vr; 
    
    epsilon = 0.15;
    vCurrent = LSimoveC_UC_MoveCtrl_NPivot_W2V(n, a.pivotValues, epsilon, a.pivotParam);
    g.vCurrent(:,i) = vCurrent; 
%     vCurrent = v; 

elseif strcmp(agvType, 'Mecanum')
    r = 1;%wheeldiameter/2; 
    L = 1.5;%ly + lx
    J = 1/r.*[-1 1 L; 1 1 L; -1 -1 L; 1 -1 L]; 
    phi = J*vSet';
    vCurrent = pinv(J)*phi;
    
    a.statMovement = true; 
    
elseif strcmp(agvType, 'SteeredDiff_OneWheel')
    
    [vs, a.betaSetpoints, steering, movement] = LSimoveC_UC_Kin_Tricycle_V2W(vRamped, a.pivotParam(1), a.oldBeta(1), control.pivot.angularTolerance, [pi/2*1.05, -pi/2*1.05]); 
   
    if abs(vs) > velocity.vsmax
        vRamped = vRamped .* (velocity.vsmax / vs);
        [vs, a.betaSetpoints, steering, movement] = LSimoveC_UC_Kin_Tricycle_V2W(vRamped, a.pivotParam(1), a.oldBeta(1), control.pivot.angularTolerance, [pi/2*1.05, -pi/2*1.05]); 
    end
    
    % SETPOINT
    releaseSteering = 1;
    steerShortestWay = false;
    setpointSteeringVelocity = LSimoveC_UC_Kin_SteeringControl...
        (releaseSteering, a.betaSetpoints(1), a.oldBeta(1), steerShortestWay, control.pivot.omegaMaxRadS, control.pivot.maxAccelerationWheel, control.pivot.kp, samplingTime);

%     [steeringVelocity, a.z] = myFilter(motor(1,:),motor(2,:),setpointSteeringVelocity,a.z); % transfer function
    steeringVelocity = setpointSteeringVelocity; % tf = 1
    beta = a.oldBeta(1) + steeringVelocity * samplingTime;
    a.pivotValues(1).angularPosition = beta;
    
    vCurrent = LSimoveC_UC_MoveCtrl_Tricycle_W2V(vs, beta(1), a.pivotParam(1)); 
    
    [vsPivot, betaPivot] = LSimoveC_UC_MoveCtrl_NPivot_V2W(vCurrent, a.pivotValues, a.pivotParam, n, 99, 1, pi, 0.5, 0, 0);
    
    [vlDiff, vrDiff] = invKinDiffDrive(vCurrent,pivotParam(2)); 
    
    if abs(beta) < 1e-3
        phiBetaCoeff = 1;
    else
        phiBetaCoeff = abs(a.pivotParam(1).position.x*tan(pi/2 - beta)+pivotParam(2).ringmount) / sqrt((a.pivotParam(1).position.x*tan(pi/2 - beta))^2+a.pivotParam(1).position.x^2);
    end
    vrDiff = vrDiff * 2*pi * pivotParam(2).diameter/2.0 / 60.0 / pivotParam(2).gearratio;
    
    vCurrent2(1) = vrDiff / phiBetaCoeff * cos(beta);
    vCurrent2(2) = 0;
    vCurrent2(3) = vrDiff / a.pivotParam(1).position.x / phiBetaCoeff * sin(beta);
    
    a.statSteering = steering; 
    a.statMovement = movement;
    a.oldBeta = beta;
    
    a.vsCurrentBuffer(i) = vs; 
    a.betaCurrentBuffer(i) = beta; 
    a.betaSetpointBuffer(i) = a.betaSetpoints;
    a.steeringVelocitySetpointBuffer(i) = setpointSteeringVelocity;
    a.steeringVelocityBuffer(i) = steeringVelocity; 
    a.steeringBuffer(i) = steering;
    a.movementBuffer(i) = movement; 
    
else
    disp('No W2V for such AGV type!')
end

%% Update AGV position
g.curMapPos(3) = g.oldMapPos(3) + vCurrent(3) * samplingTime;
% if curMapPos(3) < 0 
%     curMapPos(3) = curMapPos(3) + 2*pi; 
% end
% if curMapPos(3) > 2*pi
%     curMapPos(3) = curMapPos(3) - 2*pi;
% end

g.curMapPos(1) = g.oldMapPos(1) ...
                       + (vCurrent(1) * samplingTime) * cos(g.curMapPos(3) ) ...
                       - (vCurrent(2) * samplingTime) * sin(g.curMapPos(3) ); 
g.curMapPos(2) = g.oldMapPos(2) ...
                       + (vCurrent(1) * samplingTime) * sin(g.curMapPos(3) ) ...
                       + (vCurrent(2) * samplingTime) * cos(g.curMapPos(3) ); 

if strcmp(agvType, 'TricycleRear') || strcmp(agvType, 'SteeredDiff_OneWheel')
    
    alternateMapPos(3) = alternateMapPosOld(3) + vCurrent2(3) * samplingTime;

    alternateMapPos(1) = alternateMapPosOld(1) ...
                           + (vCurrent2(1) * samplingTime) * cos(alternateMapPos(3) ) ...
                           - (vCurrent2(2) * samplingTime) * sin(alternateMapPos(3) ); 
    alternateMapPos(2) = alternateMapPosOld(2) ...
                           + (vCurrent2(1) * samplingTime) * sin(alternateMapPos(3) ) ...
                           + (vCurrent2(2) * samplingTime) * cos(alternateMapPos(3) ); 
                       
    alternateMapPosOld = alternateMapPos;
end
                   
if strcmp(sensor.type, 'ANS')
    % ERROR SCENARIO:
    % Track offset of 1cm on the last 20cm before the target
    errorScenario = 0; 
    if targetLength < 0.2 && errorScenario == 1 && ~errorBit 
        g.curMapPos(2) = g.curMapPos(2) + 0.01;
        errorIndex = i; 
        errorBit = true; 
    end
end
g.oldMapPos = g.curMapPos;

% toc
%% PLOT everything
if mod(i,100) == 0 && enablePlots
    
    if j == 0 
        fGlobalMap = figure('units','normalized','outerposition',[0 0 1 1]);
    end
    j = j + 1;
    
    pause(0.001)
    
    figure(fGlobalMap); clf
    plot(xq,yq,'LineWidth',2); grid on; hold on 
    plotAGV(a,g)

    % AGV position trace                               
    agvCenter(j,:) = [g.curMapPos(1) g.curMapPos(2)]; 
    plot(agvCenter(1:j,1),agvCenter(1:j,2))        
    plot(xq(endIDindx),yq(endIDindx),'go')
    
    % sensor
    if strcmp(sensor.type, 'ANS')
        plot(tdPoint(1),tdPoint(2),'bx') % projection of AGV position onto track
        plotVTS(carrotGlobal)   
    elseif strcmp(sensor.type, 'Y') || strcmp(sensor.type, 'YA')
        plot(sensorFrame(2:3,1), sensorFrame(2:3,2),'k-','LineWidth',1.5) % AGV sensor
        plot(xi, yi,'ro') % plot intersection
    end

    axis([min(xq)-1 max(xq)+1 min(yq)-1 max(yq)+1])
    axis equal
    title('Global Map')
    ylim=get(gca,'ylim');
    xlim=get(gca,'xlim');

    if strcmp(sensor.type, 'Y')
        content = sprintf([  'TrackSpeed: ', num2str( sqrt(vCurrent(1)^2 + vCurrent(2)^2) ), ' m/s' ...
                           '\nP_Y: '       , num2str( control.y.pTrack )... 
                           '\nI_Y: '       , num2str( control.y.iTrack )... 
                           '\nD_Y: '       , num2str( control.y.dTrack )... 
                           '\nvA_{max}: '  , num2str( max(abs(g.vCurrent(3,:))) ), ' rad/s' ...
                           '\nyDev_{max}: ', num2str( max(abs(g.trackDeviationBuffer(2,:)))*1e3 ), ' mm' ... 
                           '\npos.x: '     , num2str( sensor.pos(1)*1e3 ), ' mm' ...
                             ]);
    elseif strcmp(sensor.type, 'YA')
        content = sprintf([  'TrackSpeed: ', num2str( sqrt(vCurrent(1)^2 + vCurrent(2)^2) ), ' m/s' ...
                           '\nP_Y: '       , num2str( control.y.pTrack ), '   P_A: ', num2str( control.a.pTrack )... 
                           '\nI_Y: '       , num2str( control.y.iTrack ), '     I_A: ', num2str( control.a.iTrack )... 
                           '\nD_Y: '       , num2str( control.y.dTrack ), '   D_A: ', num2str( control.a.dTrack )...  
                           '\nvA_{max}: '  , num2str( max(abs(g.vCurrent(3,:))) ), ' rad/s' ...
                           '\nyDev_{max}: ', num2str( max(abs(g.trackDeviationBuffer(2,:)))*1e3 ), ' mm' ... 
                           '\npos.x: '     , num2str( sensor.pos(1)*1e3 ), ' mm' ...
                             ]);
        
    elseif strcmp(sensor.type, 'ANS')
        content = sprintf([  'TrackSpeed: ', num2str( sqrt(vCurrent(1)^2 + vCurrent(2)^2) ), ' m/s' ...
                           '\nvA_{max}: '  , num2str( max(abs(g.vCurrent(3,:))) ), ' rad/s' ...
                           '\nyDev_{max}: ', num2str( max(abs(g.trackDeviationBuffer(2,:)))*1e3 ), ' mm' 
                             ]);  
                         
    else
        content = 'sensortype unknown';
    end
    text(xlim(2)-2,ylim(1)+2,content)
    
    if strcmp(agvType, 'SteeredDiff_SteerMaster') || strcmp(agvType, 'SteeredDiff_DiffMaster') || ...
            strcmp(agvType, 'SteeredDiff_OneWheel')
        
        plot(alternateMapPos(1), alternateMapPos(2),'b+','LineWidth',1.25); hold on % AGV center

    elseif strcmp(agvType, 'Tricycle')
        
        ey = sqrt( (target(2)*cos(target(3)-pi/2)/sin(target(3)-pi/2))^2 + target(2)^2 ); % ytolerance in [m]
        content = sprintf([  'Track Speed: ', num2str( sqrt(vCurrent(1)^2 + vCurrent(2)^2) ), ' m/s',               ...
                           '\nWheel Omega_{max}: ', num2str(control.pivot.omegaMaxRadS), ' rad/s',                  ...
                           '\nWheel Acceleration_{max}: ', num2str(control.pivot.maxAccelerationWheel), ' rad/s^2', ...
                           '\nK_p Steering: ', num2str(control.pivot.kp),                                           ...
                           '\ne_y: ', num2str(ey),                                                                  ...
                           '\nTrackSpeed: ', num2str( sqrt(vCurrent(1)^2 + vCurrent(2)^2) ), ' m/s' 
                           ]);
        text(xlim(2)-2,ylim(1)+2,content)
        
    elseif strcmp(agvType, 'TricycleRear')

        plot(alternateMapPos(1), alternateMapPos(2),'b+','LineWidth',1.25); hold on % AGV center
        
        content = sprintf([  'Wheel Omega_{max}: ', num2str(control.pivot.omegaMaxRadS), ' rad/s',                  ...
                           '\nWheel Acceleration_{max}: ', num2str(control.pivot.maxAccelerationWheel), ' rad/s^2', ...
                           '\nK_p Steering: ', num2str(control.pTrack), ...
                           '\nK_p Track: ', num2str(control.pTrack), ...
                           '\nvs: ', num2str( vs ), ' m/s' 
                           ]);
        text(xlim(2)-2,ylim(1)+2,content)
        
    elseif strcmp(agvType, 'Mecanum') || strcmp(agvType, 'Diff') || strcmp(agvType, 'PivotPassive')           
        % nothing  
    else 
        figure(fGlobalMap); clf
        j = 1; % to avoid endless windows popping up
    end
end

end
%% ANS Target
if strcmp(sensor.type, 'ANS')

    figure('units','normalized','outerposition',[0 0 1 1])
    subplot(311) 
    plot(1:i, g.trackDeviationBuffer(1,1:i)); hold on; grid on 
    ylim = get(gca,'ylim');
    if errorIndex > 0; plot([errorIndex errorIndex],[ylim(1) ylim(2)],'k-.','LineWidth',1); end
    h = fill([positioningIndex,i,i,positioningIndex],...
             [ylim(1),ylim(1),ylim(2),ylim(2)],'green');
    set(h,'Linestyle','none','facealpha',.3)
    title('Target X')
    if errorIndex > 0; legend('Target_X','Error','Positioning'); 
    else; legend('Target_X','Positioning'); end

    subplot(312)
    plot(1:i, g.trackDeviationBuffer(2,1:i)); hold on; grid on
    ylim = get(gca,'ylim');
    if errorIndex > 0; plot([errorIndex errorIndex],[ylim(1) ylim(2)],'k-.','LineWidth',1); end
    h = fill([positioningIndex,i,i,positioningIndex],...
             [ylim(1),ylim(1),ylim(2),ylim(2)],'green');
    set(h,'Linestyle','none','facealpha',.3)
    title('Target Y')
    if errorIndex > 0; legend('Target_Y','Error','Positioning');
    else; legend('Target_Y','Positioning'); end

    subplot(313) 
    plot(1:i, g.trackDeviationBuffer(3,1:i)); hold on; grid on
    ylim = get(gca,'ylim');
    if errorIndex > 0; plot([errorIndex errorIndex],[ylim(1) ylim(2)],'k-.','LineWidth',1); end
    h = fill([positioningIndex,i,i,positioningIndex],...
             [ylim(1),ylim(1),ylim(2),ylim(2)],'green');
    set(h,'Linestyle','none','facealpha',.3)
    title('Target A')
    if errorIndex > 0; legend('Target_A','Error','Positioning');
    else; legend('Target_A','Positioning'); end
    
    if sensor.ans.targetXValue > 0
        figure    
        subplot(211)   
        plot(1:i-1, propPartY(1:i-1)); grid on; hold on 
        plot(1:i-1, intePartY(1:i-1)) 
        plot(1:i-1, diffPartY(1:i-1))
        plot(1:i-1, propPartY(1:i-1)+intePartY(1:i-1)+diffPartY(1:i-1),'LineWidth',2.0)
        axis([0 i min(propPartY(1:i-1)+intePartY(1:i-1)+diffPartY(1:i-1))-0.01 max(propPartY(1:i-1)+intePartY(1:i-1)+diffPartY(1:i-1))+0.01])
        title('PID_Y')
        
        subplot(212)
        plot(1:i-1, propPartA(1:i-1)); grid on; hold on 
%         plot(1:i-1, intePartA(1:i-1)) 
        plot(1:i-1, diffPartA(1:i-1))
%         plot(1:i-1, propPartA(1:i-1)+intePartA(1:i-1)+diffPartA(1:i-1),'LineWidth',2.0)
        axis([0 i min(propPartA(1:i-1)+intePartA(1:i-1)+diffPartA(1:i-1))-0.01 max(propPartA(1:i-1)+intePartA(1:i-1)+diffPartA(1:i-1))+0.01])
        title('PID_A')
        legend('1','2','3','4')
    end

elseif strcmp(sensor.type, 'Y')
    
    figure('units','normalized','outerposition',[0 0 1 1])
    subplot(211)
    plot(1:i, g.trackDeviationBuffer(2,1:i)); grid on 
    title('Track Deviation in [m]')
    axis([0 i min(g.trackDeviationBuffer(2,1:i))-0.01 max(g.trackDeviationBuffer(2,1:i))+0.01])   
    subplot(212)
    title('PID')
    plot(1:i-1, propPart(1:i-1)); grid on; hold on 
    plot(1:i-1, intePart(1:i-1)) 
    plot(1:i-1, diffPart(1:i-1))
    plot(1:i-1, propPart(1:i-1)+intePart(1:i-1)+diffPart(1:i-1),'LineWidth',2.0)
    axis([0 i min(propPart(1:i-1)+intePart(1:i-1)+diffPart(1:i-1))-0.01 max(propPart(1:i-1)+intePart(1:i-1)+diffPart(1:i-1))+0.01])
    
elseif strcmp(sensor.type, 'YA')
    
    figure('units','normalized','outerposition',[0 0 1 1])
    subplot(211)
    plot(1:i, g.trackDeviationBuffer(2,1:i)); grid on 
    title('Track Deviation in [m]')
    axis([0 i min(g.trackDeviationBuffer(2,1:i))-0.01 max(g.trackDeviationBuffer(2,1:i))+0.01])   
    subplot(212)
    plot(1:i, g.trackDeviationBuffer(3,1:i)); grid on 
    title('Track Deviation in [rad]')
    axis([0 i min(g.trackDeviationBuffer(3,1:i))-0.01 max(g.trackDeviationBuffer(3,1:i))+0.01])   
end 

%% XYA Velocities
plotXYAVelocities(g,i)

%% Process Values
if strcmp(agvType, 'Diff')
    figure('units','normalized','outerposition',[0 0 1 1])
    subplot(311)
    plot(1:i, a.vlBuffer(1:i)); hold on; grid on
    title('v_L')

    subplot(312)
    plot(1:i, a.vrBuffer(1:i)); hold on; grid on
    title('v_R')
    
elseif strcmp(agvType, 'Tricycle') || strcmp(agvType, 'TricycleRear') || ... 
        strcmp(agvType, 'SteeredDiff_SteerMaster') || strcmp(agvType, 'SteeredDiff_DiffMaster')
    figure('units','normalized','outerposition',[0 0 1 1])
    subplot(311)
    plot(1:i, a.vsCurrentBuffer(1:i)); hold on; grid on
    ylim = get(gca,'ylim');
    if errorIndex > 0; plot([errorIndex errorIndex],[ylim(1) ylim(2)],'k-.','LineWidth',1); end
    if strcmp(sensor.type, 'ANS')
        h = fill([positioningIndex,i,i,positioningIndex],...
                 [ylim(1),ylim(1),ylim(2),ylim(2)],'green');
        set(h,'Linestyle','none','facealpha',.3)
    end
    title('vs')
    if ~strcmp(sensor.type, 'ANS'); legend('vs');
    elseif errorIndex > 0; legend('vs','Error','Positioning');
    else; legend('vs','Positioning'); 
    end

    subplot(312)
    plot(1:i, a.betaCurrentBuffer(1:i)*180/pi); grid on; hold on 
    plot(1:i, a.betaSetpointBuffer(1:i)*180/pi)
    ylim=get(gca,'ylim');
    if errorIndex > 0; plot([errorIndex errorIndex],[ylim(1) ylim(2)],'k-.','LineWidth',1); end
    if strcmp(sensor.type, 'ANS')
        h= fill([positioningIndex,i,i,positioningIndex],...
                [ylim(1),ylim(1),ylim(2),ylim(2)],'green');
        set(h,'Linestyle','none','facealpha',.3)
    end
    title('beta')
    if ~strcmp(sensor.type, 'ANS'); legend('beta');
    elseif errorIndex > 0; legend('beta','betaSetpoint','Error','Positioning');
    else; legend('beta','betaSetpoint','Positioning'); 
    end

    subplot(313)
    plot(1:i, a.steeringVelocityBuffer(1:i)); hold on, grid on 
    plot(1:i, a.steeringVelocitySetpointBuffer(1:i))
    ylim=get(gca,'ylim');
    if errorIndex > 0; plot([errorIndex errorIndex],[ylim(1) ylim(2)],'k-.','LineWidth',1); end
    if strcmp(sensor.type, 'ANS')
        h= fill([positioningIndex,i,i,positioningIndex],...
                [ylim(1),ylim(1),ylim(2),ylim(2)],'green');
        set(h,'Linestyle','none','facealpha',.3)
    end
    title('beta velocity')
    if ~strcmp(sensor.type, 'ANS'); legend('betaVelocitySetpoint');
    elseif errorIndex > 0; legend('betaVelocitySetpoint','betaVelocity','Error','Positioning');
    else; legend('betaVelocitySetpoint','betaVelocity','Positioning'); 
    end
    
elseif strcmp(agvType, 'PivotActive')
    figure('units','normalized','outerposition',[0 0 1 1])
    
    subplot(211)
    plot(1:i, a.vsSetpointBuffer(1,1:i)); hold on; grid on
    plot(1:i, a.vsSetpointBuffer(2,1:i))
    plot(1:i, a.vsSetpointBuffer(3,1:i))
    legend('front','left','right')
    title('v_S')

    subplot(212)
    plot(1:i, a.betaCurrentBuffer(1,1:i)); hold on; grid on
    plot(1:i, a.betaCurrentBuffer(2,1:i))
    plot(1:i, a.betaCurrentBuffer(3,1:i))
    legend('front','left','right')
    title('beta')

 elseif strcmp(agvType, 'PivotPassive')
    figure('units','normalized','outerposition',[0 0 1 1])
    
    subplot(411)
    plot(1:i, a.vsCurrentBuffer(1,1:i)); hold on; grid on
    plot(1:i, a.vsCurrentBuffer(2,1:i)); grid on 
    title('v_s')
    
    subplot(412)
    plot(1:i, a.betaCurrentBuffer(1,1:i)); hold on; grid on
    plot(1:i, a.betaCurrentBuffer(2,1:i)); grid on 
    title('alpha')
    
    subplot(413)
    plot(1:i, a.vlBuffer(1,1:i)); hold on; grid on
    plot(1:i, a.vlBuffer(2,1:i)); grid on 
    title('v_L')

    subplot(414)
    plot(1:i, a.vrBuffer(1,1:i)); hold on; grid on
    plot(1:i, a.vrBuffer(2,1:i)); grid on 
    title('v_R')
end