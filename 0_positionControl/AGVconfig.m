function [pivotParam, kinType, motor, n, agvMeasures, pivotValues, sensor, name, control, velocity] = AGVconfig (name, sampleTime, optPGainY, optDGainY)
% [pivotParam, kinType, motor, n] = AGVconfig (name, sampleTime)
%
% known AGV types ('name'): 
% - dpm_miniCart
% - Schuhschachtel_ANS
% - Schuhschachtel_YA
% - EWS_WFT
% - MaxMover
% - Tricycle_Tuenkers
% - M240
% - BMF
% - BMF_TricycleRear
% - BMF_SteeredDiff_SteerMaster
% - BMF_SteeredDiff_DiffMaster
% - Baer
% - VDL
% - Evocortex
% - Mecanum_Test
% - Eckhart
% - Eisenmann -- not printable 
    
n = 1;
kinType = 'Diff';
motor = 1; 

pivotParam(1).position.x = 0;
pivotParam(1).position.y = 0;
pivotParam(1).ringmount = 0.32;
pivotParam(1).diameter = 0.2;
pivotParam(1).gearratio = 1;

agvMeasures.front = 0.5; % in [m]
agvMeasures.back  = 0.3; % in [m]
agvMeasures.left  = 0.15; % in [m]
agvMeasures.right = 0.15; % in [m]

pivotValues(1).angularPosition = 0;
pivotValues(1).velocity = 0; 

sensor.type = 'ANS'; 
sensor.ans.targetDistance = 0.5; % in [m]

velocity.trackSpeedTarget = 1; % [m/s]

velocity.vMax.x = 1; % [m/s]
velocity.vMax.y = 1; % [m/s]
velocity.vMax.a = 1; % [rad/s]
velocity.vMax.xy = 1; 

velocity.acc.x = 1; % [m/s²]
velocity.acc.y = 1; % [m/s²]
velocity.acc.a = 1; % [rad/s²]
velocity.acc.xy = 1; 

velocity.dec.x = 1;  % [m/s²]
velocity.dec.y = 1;  % [m/s²]
velocity.dec.a = 1;  % [rad/s²]
velocity.dec.xy = 1;

control.pivot.omegaMaxRadS = 1;
control.pivot.maxAccelerationWheel = 1;
control.pivot.kp = 0.002; 
control.pivot.angularTolerance = pi;

sensor.ans.targetXValue = -1; % in [m] -- "-1" is astolfi controller
control.y.pTrack = 1.0;
control.y.iTrack = 0.0;
control.y.dTrack = 0.0;

control.a.pTrack = 1.0;
control.a.iTrack = 0.0;
control.a.dTrack = 1.0;

if strcmp(name, 'Schuhschachtel_YA')
    
    load motorWFT.mat tfunction 
    % motorWFT = c2d(tfunction,sampleTime);
    motorWFT = c2d(tf(1),sampleTime);
    [num,den] = tfdata(motorWFT,'v');
    motor = [num; den];
    
    n = 1;
    kinType = 'Diff';
    
    pivotParam(1).position.x = 0;
    pivotParam(1).position.y = 0;
    pivotParam(1).ringmount = 0.32;
    pivotParam(1).diameter = 0.2;
    pivotParam(1).gearratio = 1;
    
    agvMeasures.front = 0.2; % in [m]
    agvMeasures.back  = 0.35; % in [m]
    agvMeasures.left  = 0.2; % in [m]
    agvMeasures.right = 0.2; % in [m]
    
    sensor.type = 'YA';
    sensor.pos(1) = 0.2;
    sensor.pos(2) = 0.0;
    sensor.range = .517;
    
    velocity.trackSpeed = 0.25;
    velocity.trackSpeedTarget = 0.25;
    
    velocity.vMax.x = 2; % [m/s]
    velocity.vMax.y = 0; % [m/s]
    velocity.vMax.a = 200; % [rad/s]
    velocity.vMax.xy = 3; 
    
    velocity.acc.x = 1; % [m/s²]
    velocity.acc.y = 0; % [m/s²]
    velocity.acc.a = 300; % [rad/s²]
    velocity.acc.xy = 3; 

    velocity.dec.x = 0.8;  % [m/s²]
    velocity.dec.y = 0;  % [m/s²]
    velocity.dec.a = 400;  % [rad/s²]
    velocity.dec.xy = 2;
    
    control.y.pTrack = 10;
    control.y.iTrack = 0.0;
    control.y.dTrack = 0.0;
    
    control.a.pTrack = 0;   %2
    control.a.iTrack = 0;
    control.a.dTrack = 0; 
        
    if nargin > 2 
        control.y.pTrack = optPGainY;
    end 

    if nargin > 3
        control.y.pTrack = optDGainY; 
    end
elseif strcmp(name, 'Schuhschachtel_YA_Niklas')     %kopiert von Schuhschachtel_YA
    
    load motorWFT.mat tfunction 
    motorWFT = c2d(tfunction,sampleTime);
%     motorWFT = c2d(tf(1),sampleTime);
    [num,den] = tfdata(motorWFT,'v');
    motor = [num; den];
    
    n = 1;
    kinType = 'Diff';
    
    pivotParam(1).position.x = 0;
    pivotParam(1).position.y = 0;
    pivotParam(1).ringmount = 0.32;
    pivotParam(1).diameter = 0.2;
    pivotParam(1).gearratio = 1;
    
    agvMeasures.front = 0.2; % in [m]
    agvMeasures.back  = 0.35; % in [m]
    agvMeasures.left  = 0.2; % in [m]
    agvMeasures.right = 0.2; % in [m]
    
    sensor.type = 'YA';
    sensor.pos(1) = 0.2;
    sensor.pos(2) = 0.0;
    sensor.range = .517;
    
    velocity.trackSpeed = 0.25;
    velocity.trackSpeedTarget = 0.25;
    
    velocity.vMax.x = 2; % [m/s]
    velocity.vMax.y = 0; % [m/s]
    velocity.vMax.a = 2; % [rad/s]
    velocity.vMax.xy = 3; 
    
    velocity.acc.x = 1; % [m/s²]
    velocity.acc.y = 0; % [m/s²]
    velocity.acc.a = 3; % [rad/s²]
    velocity.acc.xy = 3; 

    velocity.dec.x = 0.8;  % [m/s²]
    velocity.dec.y = 0;  % [m/s²]
    velocity.dec.a = 4;  % [rad/s²]
    velocity.dec.xy = 2;
    
    control.y.pTrack = 5;
    control.y.iTrack = 0.0;
    control.y.dTrack = 0.0;
    
    control.a.pTrack = 2;
    control.a.iTrack = 0;
    control.a.dTrack = 0; 
        
    if nargin > 2 
        control.y.pTrack = optPGainY;
    end 

    if nargin > 3
        control.y.pTrack = optDGainY; 
    end

elseif strcmp(name, 'EWS_WFT')
    load motorWFT.mat tfunction 
    motorWFT = c2d(tfunction,sampleTime);
%     motorWFT = c2d(tf(1),sampleTime);
    [num,den] = tfdata(motorWFT,'v');
    motor = [num; den];
    
    n = 1;
    kinType = 'Tricycle';
    agvMeasures.front = 1.2; % in [m]
    agvMeasures.back  = 0.3; % in [m]
    agvMeasures.left  = 0.5; % in [m]
    agvMeasures.right = 0.5; % in [m]
    pivotParam(1).position.x = 0.8;  % in [m]
    pivotParam(1).position.y = 0.0;  % in [m]
    pivotParam(1).ringmount = 0.2; % in [m]
    pivotParam(1).diameter = 0.2;  % in [m]
    pivotParam(1).gearratio = 1;
    
    pivotValues(1).angularPosition = 0;
    pivotValues(1).velocity = 0; 
    
    sensor.type = 'YA';  
    sensor.pos(1) = 1;
    sensor.pos(2) = 0.0;
    sensor.range = 1;
    
    sensor.ans.targetDistance = 1.5; % in [m]
    sensor.ans.targetXValue = 0.5; % in [m] -- "-1" is astolfi controller
    
    velocity.trackSpeed = 0.5; % [m/s]

    velocity.vMax.x = 2; % [m/s]
    velocity.vMax.y = 0.7; % [m/s]
    velocity.vMax.a = 2; % [rad/s]
    velocity.vMax.xy = 3; 
    
    velocity.acc.x = 1; % [m/s²]
    velocity.acc.y = 0.5; % [m/s²]
    velocity.acc.a = 3; % [rad/s²]
    velocity.acc.xy = 3; 

    velocity.dec.x = 0.8;  % [m/s²]
    velocity.dec.y = 0.8;  % [m/s²]
    velocity.dec.a = 4;  % [rad/s²]
    velocity.dec.xy = 2;
    
    control.pivot.omegaMaxRadS = 0.5;
    control.pivot.maxAccelerationWheel = 4;
    control.pivot.kp = 0.2; 
    control.pivot.angularTolerance = pi; % in [rad] -- no initial steering, to avoid unnecessary stand rotations
    
    control.y.pTrack = 2.5;
    control.y.iTrack = 0.0;
    control.y.dTrack = 0.0;
    
    control.a.pTrack = 0;
    control.a.iTrack = 0;
    control.a.dTrack = 0; 
      
elseif strcmp(name, 'Baer')
    
    motorM240 = tf(1);
    [num,den] = tfdata(motorM240,'v');
    motor = [num; den];
    
    n = 2; 
    kinType = 'PivotActive';
    pivotParam(1).position.x = 0.7;
    pivotParam(1).position.y = 0.2;
    pivotParam(1).ringmount = 0;
    pivotParam(1).diameter = 0.2;
    pivotParam(1).gearratio = 1;
    pivotParam(1).forbiddenAngles(1).higher = 181; 
    pivotParam(1).forbiddenAngles(1).lower = 120; 
    pivotParam(1).forbiddenAngles(2).higher = -120; 
    pivotParam(1).forbiddenAngles(2).lower = -181; 

    pivotParam(2).position.x = -0.7;
    pivotParam(2).position.y = -0.2;
    pivotParam(2).ringmount = 0;
    pivotParam(2).diameter = 0.2;
    pivotParam(2).gearratio = 1;
    pivotParam(2).forbiddenAngles(1).higher = 181; 
    pivotParam(2).forbiddenAngles(1).lower = 120; 
    pivotParam(2).forbiddenAngles(2).higher = -120; 
    pivotParam(2).forbiddenAngles(2).lower = -181; 
    
    pivotValues(1).angularPosition = 0; 
    pivotValues(1).velocity = 0; 
    pivotValues(2).angularPosition = 0; 
    pivotValues(2).velocity = 0; 
    
    agvMeasures.front = 1; % in [m]
    agvMeasures.back  = 1; % in [m]
    agvMeasures.left  = 0.5; % in [m]
    agvMeasures.right = 0.5; % in [m]
    
    sensor.type = 'YA';
    sensor.pos(1) = 0.0;
    sensor.pos(2) = 0.1;
    sensor.range = 0.6; 
    
    velocity.trackSpeed = 0.5; % [m/s]

    velocity.vMax.x = 2; % [m/s]
    velocity.vMax.y = 0.7; % [m/s]
    velocity.vMax.a = 2; % [rad/s]
    velocity.vMax.xy = 3; 
    
    velocity.acc.x = 1; % [m/s²]
    velocity.acc.y = 0.5; % [m/s²]
    velocity.acc.a = 3; % [rad/s²]
    velocity.acc.xy = 3; 

    velocity.dec.x = 0.8;  % [m/s²]
    velocity.dec.y = 0.8;  % [m/s²]
    velocity.dec.a = 4;  % [rad/s²]
    velocity.dec.xy = 2;

elseif strcmp(name, 'normDiffPGV')
    load motorWFT.mat tfunction 
    motorWFT = c2d(tfunction,sampleTime);
    [num,den] = tfdata(motorWFT,'v');
    motor = [num; den];
    
    n = 1;
    kinType = 'Diff';
    agvMeasures.front = 1; % in [m]
    agvMeasures.back  = 0.5; % in [m]
    agvMeasures.left  = 0.4; % in [m]
    agvMeasures.right = 0.4; % in [m]
    pivotParam(1).position.x = 0;  % in [m]
    pivotParam(1).position.y = 0;  % in [m]
    pivotParam(1).ringmount = 0.2; % in [m]
    pivotParam(1).diameter = 0.2;  % in [m]
    pivotParam(1).gearratio = 1;
    
    pivotValues(1).angularPosition = 0;
    pivotValues(1).velocity = 0; 
    
    sensor.type = 'YA';  
    sensor.pos(1) = 0.5;
    sensor.pos(2) = 0.0;
    sensor.range = 0.5;
    
    velocity.trackSpeed = 0.5; % [m/s]

elseif strcmp(name, 'normTriPGV')
    load motorWFT.mat tfunction 
    motorWFT = c2d(tfunction,sampleTime);
    [num,den] = tfdata(motorWFT,'v');
    motor = [num; den];
    
    n = 1;
    kinType = 'Tricycle';
    agvMeasures.front = 1; % in [m]
    agvMeasures.back  = 0.5; % in [m]
    agvMeasures.left  = 0.4; % in [m]
    agvMeasures.right = 0.4; % in [m]
    pivotParam(1).position.x = 0.75;  % in [m]
    pivotParam(1).position.y = 0;  % in [m]
    pivotParam(1).ringmount = 0.2; % in [m]
    pivotParam(1).diameter = 0.2;  % in [m]
    pivotParam(1).gearratio = 1;
    
    pivotValues(1).angularPosition = 0;
    pivotValues(1).velocity = 0; 
    
    sensor.type = 'YA';  
    sensor.pos(1) = 0.5;
    sensor.pos(2) = 0.0;
    sensor.range = 0.5;
    
    velocity.trackSpeed = 0.5; % [m/s]

    control.pivot.kp = 0.01; % 0.05 ist instabil auf sCurve 

elseif strcmp(name, 'normNpiPGV')
    load motorWFT.mat tfunction 
    motorWFT = c2d(tfunction,sampleTime);
    [num,den] = tfdata(motorWFT,'v');
    motor = [num; den];
    
    agvMeasures.front = 0.75; % in [m]
    agvMeasures.back  = 0.75; % in [m]
    agvMeasures.left  = 0.4; % in [m]
    agvMeasures.right = 0.4; % in [m]
    
    n = 2; 
    kinType = 'PivotPassive';
    pivotParam(1).position.x = 0.5;
    pivotParam(1).position.y = 0.2;
    pivotParam(1).ringmount = 0.1;
    pivotParam(1).diameter = 0.2;
    pivotParam(1).gearratio = 1;
    pivotParam(1).forbiddenAngles(1).higher = 181; 
    pivotParam(1).forbiddenAngles(1).lower = 120; 
    pivotParam(1).forbiddenAngles(2).higher = -120; 
    pivotParam(1).forbiddenAngles(2).lower = -181; 

    pivotParam(2).position.x = -0.5;
    pivotParam(2).position.y = -0.2;
    pivotParam(2).ringmount = 0.1;
    pivotParam(2).diameter = 0.2;
    pivotParam(2).gearratio = 1;
    pivotParam(2).forbiddenAngles(1).higher = 181; 
    pivotParam(2).forbiddenAngles(1).lower = 120; 
    pivotParam(2).forbiddenAngles(2).higher = -120; 
    pivotParam(2).forbiddenAngles(2).lower = -181; 
    
    pivotValues(1).angularPosition = 0; 
    pivotValues(1).velocity = 0; 
    pivotValues(2).angularPosition = 0; 
    pivotValues(2).velocity = 0; 
    
    sensor.type = 'YA';     
    sensor.pos(1) = 0.5;
    sensor.pos(2) = 0.0;
    sensor.range = 0.5;
    
    velocity.trackSpeed = 0.5; % [m/s]

    control.pivot.kp = 0.01; 
else
    disp('This AGV type is not known')
end

end 