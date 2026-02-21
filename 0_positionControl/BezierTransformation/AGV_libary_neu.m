function  AGV_libary_neu
%   Detailed explanation goes here

%% Schuhschachtel
agv(1).name='Schuhschachtel';

% load motorWFT.mat tfunction
%     motorWFT = c2d(tfunction,sampleTime);
%     [num,den] = tfdata(motorWFT,'v');
agv(1).motor = [ 0   -0.0001    0.0002; ...
    1.0000   -1.9927    0.9928];


agv(1).n = 1;
agv(1).type = 'Diff';

agv(1).pivotParam(1).position.x = 0;
agv(1).pivotParam(1).position.y = 0;
agv(1).pivotParam(1).ringmount = 0.32;
agv(1).pivotParam(1).diameter = 0.2;
agv(1).pivotParam(1).gearratio = 1;

agv(1).agvMeasures.front = 0.5; % in [m]
agv(1).agvMeasures.back  = 0.3; % in [m]
agv(1).agvMeasures.left  = 0.15; % in [m]
agv(1).agvMeasures.right = 0.15; % in [m]

agv(1).sensor.type = 'ANS';
agv(1).sensor.ans.targetDistance = 1;%0.5; % in [m]

agv(1).velocity.trackSpeedTarget = 1;% 0.5; % [m/s]

agv(1).velocity.vMax.x = 2; % [m/s]
agv(1).velocity.vMax.y = 0.7; % [m/s]
agv(1).velocity.vMax.a = 2; % [rad/s]
agv(1).velocity.vMax.xy = 3;

agv(1).velocity.acc.x = 1; % [m/s²]
agv(1).velocity.acc.y = 0.5; % [m/s²]
agv(1).velocity.acc.a = 3; % [rad/s²]
agv(1).velocity.acc.xy = 3;

agv(1).velocity.dec.x = 2.0;  % [m/s²]
agv(1).velocity.dec.y = 0.8;  % [m/s²]
agv(1).velocity.dec.a = 8;  % [rad/s²]
agv(1).velocity.dec.xy = 2;

agv(1).control.pivot.omegaMaxRadS = 2*2.11865346 / 180 * pi; % Geschwindigkeit berechnet von Wolfgang Umhau
agv(1).control.pivot.maxAccelerationWheel = 4;
agv(1).control.pivot.kp = 0.002;
agv(1).control.pivot.angularTolerance = pi; % in [rad] -- no initial steering, to avoid unnecessary stand rotations

%     sensor.ans.targetXValue = 1;% 0.5; % in [m] -- "-1" is astolfi controller
agv(1).sensor.ans.targetXValue = -1; % in [m] -- "-1" is astolfi controller
agv(1).control.y.pTrack = 1.0;
agv(1).control.y.iTrack = 0.0;
agv(1).control.y.dTrack = 0.0;
agv(1).control.a.pTrack = 1.0;
agv(1).control.a.iTrack = 0.0;
agv(1).control.a.dTrack = 1.0;



%% Schuhschachtel_YA

%     load motorWFT.mat tfunction
%     motorWFT = c2d(tfunction,sampleTime);
% %     motorWFT = c2d(tf(1),sampleTime);
%     [num,den] = tfdata(motorWFT,'v');
agv(2).motor = [ 0   -0.0001    0.0002; ...
    1.0000   -1.9927    0.9928];

agv(2).name= 'Schuhschachtel_YA';
agv(2).n = 1;
agv(2).type = 'Diff';

agv(2).pivotParam(1).position.x = 0;
agv(2).pivotParam(1).position.y = 0;
agv(2).pivotParam(1).ringmount = 0.32;
agv(2).pivotParam(1).diameter = 0.2;
agv(2).pivotParam(1).gearratio = 1;

agv(2).agvMeasures.front = 0.2; % in [m]
agv(2).agvMeasures.back  = 0.35; % in [m]
agv(2).agvMeasures.left  = 0.2; % in [m]
agv(2).agvMeasures.right = 0.2; % in [m]

%     pivotValues = 0;

agv(2).sensor.type = 'Y';
agv(2).sensor.pos(1) = 0.2;
agv(2).sensor.pos(2) = 0.0;
agv(2).sensor.range = 0.117;

agv(2).velocity.trackSpeed = 0.5;

agv(2).velocity.vMax.x = 2; % [m/s]
agv(2).velocity.vMax.y = 0.7; % [m/s]
agv(2).velocity.vMax.a = 2; % [rad/s]
agv(2).velocity.vMax.xy = 3;

agv(2).velocity.acc.x = 1; % [m/s²]
agv(2).velocity.acc.y = 0.5; % [m/s²]
agv(2).velocity.acc.a = 3; % [rad/s²]
agv(2).velocity.acc.xy = 3;

agv(2).velocity.dec.x = 0.8;  % [m/s²]
agv(2).velocity.dec.y = 0.8;  % [m/s²]
agv(2).velocity.dec.a = 4;  % [rad/s²]
agv(2).velocity.dec.xy = 2;

agv(2).control.y.pTrack = 2.5;
agv(2).control.y.iTrack = 0.0;
agv(2).control.y.dTrack = 0.0;

agv(2).control.a.pTrack = 0.7;
agv(2).control.a.iTrack = 0;
agv(2).control.a.dTrack = 0;

%     control.pivot.omegaMaxRadS = 2*2.11865346 / 180 * pi; % Geschwindigkeit berechnet von Wolfgang Umhau
%     control.pivot.maxAccelerationWheel = 4;
%     control.pivot.kp = 0.002;
%     control.pivot.angularTolerance = pi; % in [rad] -- no initial steering, to avoid unnecessary stand rotations

%% EWS_WFT
%
%  load motorWFT.mat tfunction
%     motorWFT = c2d(tfunction,sampleTime);
% %     motorWFT = c2d(tf(1),sampleTime);
%     [num,den] = tfdata(motorWFT,'v');
agv(3).motor = [ 0   -0.0001    0.0002; ...
    1.0000   -1.9927    0.9928];


agv(3).name ='EWS_WFT';
agv(3).n = 1;
agv(3).type = 'Tricycle';
agv(3).agvMeasures.front = 1.2; % in [m]
agv(3).agvMeasures.back  = 0.3; % in [m]
agv(3).agvMeasures.left  = 0.5; % in [m]
agv(3).agvMeasures.right = 0.5; % in [m]
agv(3).pivotParam(1).position.x = 0.8;  % in [m]
agv(3).pivotParam(1).position.y = 0.0;  % in [m]
agv(3).pivotParam(1).ringmount = 0.2; % in [m]
agv(3).pivotParam(1).diameter = 0.2;  % in [m]
agv(3).pivotParam(1).gearratio = 1;

agv(3).pivotValues(1).angularPosition = 0;
agv(3).pivotValues(1).velocity = 0;

agv(3).sensor.type = 'ANS';
agv(3).sensor.pos(1) = 0.5;
agv(3).sensor.pos(2) = 0.0;
agv(3).sensor.range = 1;

agv(3).sensor.ans.targetDistance = 1.5; % in [m]
agv(3).sensor.ans.targetXValue = 0.5; % in [m] -- "-1" is astolfi controller

agv(3).velocity.trackSpeed = 0.5; % [m/s]

agv(3).velocity.vMax.x = 2; % [m/s]
agv(3).velocity.vMax.y = 0.7; % [m/s]
agv(3).velocity.vMax.a = 2; % [rad/s]
agv(3).velocity.vMax.xy = 3;

agv(3).velocity.acc.x = 1; % [m/s²]
agv(3).velocity.acc.y = 0.5; % [m/s²]
agv(3).velocity.acc.a = 3; % [rad/s²]
agv(3).velocity.acc.xy = 3;

agv(3).velocity.dec.x = 0.8;  % [m/s²]
agv(3).velocity.dec.y = 0.8;  % [m/s²]
agv(3).velocity.dec.a = 4;  % [rad/s²]
agv(3).velocity.dec.xy = 2;

agv(3).control.pivot.omegaMaxRadS = 0.5;
agv(3).control.pivot.maxAccelerationWheel = 4;
agv(3).control.pivot.kp = 0.2;
agv(3).control.pivot.angularTolerance = pi; % in [rad] -- no initial steering, to avoid unnecessary stand rotations

agv(3).control.y.pTrack = 2.5;
agv(3).control.y.iTrack = 0.0;
agv(3).control.y.dTrack = 0.0;

agv(3).control.a.pTrack = 0;
agv(3).control.a.iTrack = 0;
agv(3).control.a.dTrack = 0;

%% EWS_WFT_Bahn

%  load motorWFT.mat tfunction
%     motorWFT = c2d(tfunction,sampleTime);
% %     motorWFT = c2d(tf(1),sampleTime);
%     [num,den] = tfdata(motorWFT,'v');
agv(4).motor = [ 0   -0.0001    0.0002; ...
    1.0000   -1.9927    0.9928];
%
agv(4).name ='EWS_WFT_Bahn';
agv(4).n = 1;
agv(4).type = 'Tricycle';
agv(4).agvMeasures.front = 1.2; % in [m]
agv(4).agvMeasures.back  = 0.3; % in [m]
agv(4).agvMeasures.left  = 0.5; % in [m]
agv(4).agvMeasures.right = 0.5; % in [m]
agv(4).pivotParam(1).position.x = 0.8;  % in [m]
agv(4).pivotParam(1).position.y = 0.0;  % in [m]
agv(4).pivotParam(1).ringmount = 0.2; % in [m]
agv(4).pivotParam(1).diameter = 0.2;  % in [m]
agv(4).pivotParam(1).gearratio = 1;

agv(4).pivotValues.angularPosition = 0;
agv(4).pivotValues.velocity = 0;

agv(4).sensor.type = 'ANS_Bahn';
agv(4).sensor.ans.targetDistance = 1; % in [m]

agv(4).velocity.vMax.x = 2; % [m/s]
agv(4).velocity.vMax.y = 0.7; % [m/s]
agv(4).velocity.vMax.a = 2; % [rad/s]
agv(4).velocity.vMax.xy = 3;

agv(4).velocity.acc.x = 1; % [m/s²]
agv(4).velocity.acc.y = 0.5; % [m/s²]
agv(4).velocity.acc.a = 3; % [rad/s²]
agv(4).velocity.acc.xy = 3;

agv(4).velocity.dec.x = 0.8;  % [m/s²]
agv(4).velocity.dec.y = 0.8;  % [m/s²]
agv(4).velocity.dec.a = 4;  % [rad/s²]
agv(4).velocity.dec.xy = 2;

%% MaxMover
%
% load motorWFT.mat tfunction
%     motorWFT = c2d(tfunction,sampleTime);
% %     motorWFT = c2d(tf(1),sampleTime);
%     [num,den] = tfdata(motorWFT,'v');
agv(5).motor = [ 0   -0.0001    0.0002; ...
    1.0000   -1.9927    0.9928];

agv(5).name ='MaxMover';
agv(5).n = 1;
agv(5).type = 'Tricycle';
agv(5).agvMeasures.front = 2.3; % in [m]
agv(5).agvMeasures.back  = 0.3; % in [m]
agv(5).agvMeasures.left  = 0.5; % in [m]
agv(5).agvMeasures.right = 0.5; % in [m]
agv(5).pivotParam(1).position.x = 1.827;  % in [m]
agv(5).pivotParam(1).position.y = -0.1279;  % in [m]
agv(5).pivotParam(1).ringmount = 0.2; % in [m]
agv(5).pivotParam(1).diameter = 0.2;  % in [m]
agv(5).pivotParam(1).gearratio = 1;

agv(5).pivotValues.angularPosition = 0;
agv(5).pivotValues.velocity = 0;

agv(5).sensor.type = 'ANS';
agv(5).sensor.ans.targetDistance = 1.5; % in [m]

agv(5).velocity.vMax.x = 2; % [m/s]
agv(5).velocity.vMax.y = 0.7; % [m/s]
agv(5).velocity.vMax.a = 2; % [rad/s]
agv(5).velocity.vMax.xy = 3;

agv(5).velocity.acc.x = 1; % [m/s²]
agv(5).velocity.acc.y = 0.5; % [m/s²]
agv(5).velocity.acc.a = 3; % [rad/s²]
agv(5).velocity.acc.xy = 3;

agv(5).velocity.dec.x = 0.8;  % [m/s²]
agv(5).velocity.dec.y = 0.8;  % [m/s²]
agv(5).velocity.dec.a = 4;  % [rad/s²]
agv(5).velocity.dec.xy = 2;


%% Tricycle_Tuenkers

%      load motorWFT.mat tfunction
% %     tfunction = tf(1);
%     motorWFT = c2d(tfunction,sampleTime);
%     [num,den] = tfdata(motorWFT,'v');
agv(6).motor = [ 0   -0.0001    0.0002; ...
    1.0000   -1.9927    0.9928];

agv(6).name ='Tricycle_Tuenkers';
agv(6).n = 1;
agv(6).type = 'Tricycle';
agv(6).agvMeasures.front = 1.2; % in [m]
agv(6).agvMeasures.back  = 0.3; % in [m]
agv(6).agvMeasures.left  = 0.5; % in [m]
agv(6).agvMeasures.right = 0.5; % in [m]
agv(6).pivotParam(1).position.x = 0.75;  % in [m]
agv(6).pivotParam(1).position.y = 0;  % in [m]
agv(6).pivotParam(1).ringmount = 0.2; % in [m]
agv(6).pivotParam(1).diameter = 0.2;  % in [m]
agv(6).pivotParam(1).gearratio = 1;

agv(6).pivotValues.angularPosition = 0;
agv(6).pivotValues.velocity = 0;

agv(6).sensor.type = 'ANS';
agv(6).sensor.ans.targetDistance = 1.5; % in [m]

agv(6).velocity.vMax.x = 2; % [m/s]
agv(6).velocity.vMax.y = 0.7; % [m/s]
agv(6).velocity.vMax.a = 2; % [rad/s]
agv(6).velocity.vMax.xy = 3;

agv(6).velocity.acc.x = 1; % [m/s²]
agv(6).velocity.acc.y = 0.5; % [m/s²]
agv(6).velocity.acc.a = 3; % [rad/s²]
agv(6).velocity.acc.xy = 3;

agv(6).velocity.dec.x = 0.8;  % [m/s²]
agv(6).velocity.dec.y = 0.8;  % [m/s²]
agv(6).velocity.dec.a = 4;  % [rad/s²]
agv(6).velocity.dec.xy = 2;

%% M240

%   motorM240 = tf(1);
%     [num,den] = tfdata(motorM240,'v');
agv(7).motor = 1;

agv(7).name= 'M240';
agv(7).n = 2;
agv(7).type = 'PivotPassive';
agv(7).pivotParam(1).position.x = 0.7;
agv(7).pivotParam(1).position.y = 0.2;
agv(7).pivotParam(1).ringmount = 0.1;
agv(7).pivotParam(1).diameter = 0.2;
agv(7).pivotParam(1).gearratio = 1;
agv(7).pivotParam(1).forbiddenAngles(1).higher = 181;
agv(7).pivotParam(1).forbiddenAngles(1).lower = 120;
agv(7).pivotParam(1).forbiddenAngles(2).higher = -120;
agv(7).pivotParam(1).forbiddenAngles(2).lower = -181;

agv(7).pivotParam(2).position.x =  0.7;
agv(7).pivotParam(2).position.y = -0.2;
agv(7).pivotParam(2).ringmount = 0.1;
agv(7).pivotParam(2).diameter = 0.2;
agv(7).pivotParam(2).gearratio = 1;
agv(7).pivotParam(2).forbiddenAngles(1).higher = 181;
agv(7).pivotParam(2).forbiddenAngles(1).lower = 120;
agv(7).pivotParam(2).forbiddenAngles(2).higher = -120;
agv(7).pivotParam(2).forbiddenAngles(2).lower = -181;

agv(7).pivotValues(1).angularPosition = 0;
agv(7).pivotValues(1).velocity = 0;
agv(7).pivotValues(2).angularPosition = 0;
agv(7).pivotValues(2).velocity = 0;

agv(7).agvMeasures.front = 1; % in [m]
agv(7).agvMeasures.back  = 1; % in [m]
agv(7).agvMeasures.left  = 0.5; % in [m]
agv(7).agvMeasures.right = 0.5; % in [m]

agv(7).sensor.type = 'ANS';
agv(7).sensor.ans.targetDistance = 1; % in [m]
agv(7).sensor.ans.targetXValue = 1; % in [m] -- "-1" is astolfi controller

agv(7).velocity.trackSpeedTarget = 0.5;

agv(7).velocity.vMax.x = 2; % [m/s]
agv(7).velocity.vMax.y = 0.7; % [m/s]
agv(7).velocity.vMax.a = 2; % [rad/s]
agv(7).velocity.vMax.xy = 3;

agv(7).velocity.acc.x = 1; % [m/s²]
agv(7).velocity.acc.y = 0.5; % [m/s²]
agv(7).velocity.acc.a = 3; % [rad/s²]
agv(7).velocity.acc.xy = 3;

agv(7).velocity.dec.x = 0.8;  % [m/s²]
agv(7).velocity.dec.y = 0.8;  % [m/s²]
agv(7).velocity.dec.a = 4;  % [rad/s²]
agv(7).velocity.dec.xy = 2;

agv(7).control.pivot.omegaMaxRadS = 0.5;
agv(7).control.pivot.maxAccelerationWheel = 4;
agv(7).control.pivot.kp = 0.2;
agv(7).control.pivot.angularTolerance = pi; % in [rad] -- no initial steering, to avoid unnecessary stand rotations

agv(7).control.y.pTrack = 2.5;
agv(7).control.y.iTrack = 0.0;
agv(7).control.y.dTrack = 0.0;

agv(7).control.a.pTrack = 0;
agv(7).control.a.iTrack = 0;
agv(7).control.a.dTrack = 0;


%% Eisemann
% motorM240 = tf(1);
%     [num,den] = tfdata(motorM240,'v');
agv(8).motor = 1;

agv(8).name ='Eisemann';
agv(8).n = 4;
agv(8).type = 'PivotPassive';
agv(8).pivotParam(1).position.x = 0.7;
agv(8).pivotParam(1).position.y = 0.2;
agv(8).pivotParam(1).ringmount = 0.1;
agv(8).pivotParam(1).diameter = 0.2;
agv(8).pivotParam(1).gearratio = 1;
agv(8).pivotParam(1).forbiddenAngles(1).higher = 181;
agv(8).pivotParam(1).forbiddenAngles(1).lower = 120;
agv(8).pivotParam(1).forbiddenAngles(2).higher = -120;
agv(8).pivotParam(1).forbiddenAngles(2).lower = -181;

agv(8).pivotParam(2) = agv(8).pivotParam(1);
agv(8).pivotParam(2).position.x =  0.7;
agv(8).pivotParam(2).position.y = -0.2;

agv(8).pivotParam(3) = agv(8).pivotParam(1);
agv(8).pivotParam(3).position.x = -0.7;
agv(8).pivotParam(3).position.y =  0.2;

agv(8).pivotParam(4) = agv(8).pivotParam(1);
agv(8).pivotParam(4).position.x = -0.7;
agv(8).pivotParam(4).position.y = -0.2;

agv(8).pivotValues(1).angularPosition = 0;
agv(8).pivotValues(1).velocity = 0;
agv(8).pivotValues(2) = agv(8).pivotValues(1);
agv(8).pivotValues(3) = agv(8).pivotValues(1);
agv(8).pivotValues(4) = agv(8).pivotValues(1);

agv(8).agvMeasures.front = 1; % in [m]
agv(8).agvMeasures.back  = 1; % in [m]
agv(8).agvMeasures.left  = 0.5; % in [m]
agv(8).agvMeasures.right = 0.5; % in [m]

agv(8).sensor.type = 'ANS';
agv(8).sensor.ans.targetDistance = 1; % in [m]
agv(8).sensor.ans.targetXValue = 1; % in [m] -- "-1" is astolfi controller

agv(8).velocity.trackSpeedTarget = 0.5;

agv(8).velocity.vMax.x = 2; % [m/s]
agv(8).velocity.vMax.y = 0.7; % [m/s]
agv(8).velocity.vMax.a = 2; % [rad/s]
agv(8).velocity.vMax.xy = 3;

agv(8).velocity.acc.x = 1; % [m/s²]
agv(8).velocity.acc.y = 0.5; % [m/s²]
agv(8).velocity.acc.a = 3; % [rad/s²]
agv(8).velocity.acc.xy = 3;

agv(8).velocity.dec.x = 0.8;  % [m/s²]
agv(8).velocity.dec.y = 0.8;  % [m/s²]
agv(8).velocity.dec.a = 4;  % [rad/s²]
agv(8).velocity.dec.xy = 2;

agv(8).control.pivot.omegaMaxRadS = 0.5;
agv(8).control.pivot.maxAccelerationWheel = 4;
agv(8).control.pivot.kp = 0.2;
agv(8).control.pivot.angularTolerance = pi; % in [rad] -- no initial steering, to avoid unnecessary stand rotations

agv(8).control.y.pTrack = 2.5;
agv(8).control.y.iTrack = 0.0;
agv(8).control.y.dTrack = 0.0;

agv(8).control.a.pTrack = 0;
agv(8).control.a.iTrack = 0;
agv(8).control.a.dTrack = 0;

%% BMF

%      motorM240 = tf(1);
%     [num,den] = tfdata(motorM240,'v');
agv(9).motor = 1;

agv(9).name = 'BMF';
agv(9).n = 3;
agv(9).type = 'PivotActive';
agv(9).pivotParam(1).position.x = 0.7;
agv(9).pivotParam(1).position.y = 0.0;
agv(9).pivotParam(1).diameter = 0.2;
agv(9).pivotParam(1).gearratio = 1;
agv(9).pivotParam(1).forbiddenAngles(1).higher = 2.2;
agv(9).pivotParam(1).forbiddenAngles(1).lower = 3.2;
agv(9).pivotParam(1).forbiddenAngles(2).higher = -3.2;
agv(9).pivotParam(1).forbiddenAngles(2).lower = -2.2;

agv(9).pivotParam(2).position.x = 0.0;
agv(9).pivotParam(2).position.y = 0.3;
agv(9).pivotParam(2).diameter = 0.2;
agv(9).pivotParam(2).gearratio = 1;
agv(9).pivotParam(2).forbiddenAngles(1).higher = 2.2;
agv(9).pivotParam(2).forbiddenAngles(1).lower = 3.2;
agv(9).pivotParam(2).forbiddenAngles(2).higher = -3.2;
agv(9).pivotParam(2).forbiddenAngles(2).lower = -2.2;

agv(9).pivotParam(3).position.x = 0.0;
agv(9).pivotParam(3).position.y = -0.3;
agv(9).pivotParam(3).diameter = 0.2;
agv(9).pivotParam(3).gearratio = 1;
agv(9).pivotParam(3).forbiddenAngles(1).higher = 2.2;
agv(9).pivotParam(3).forbiddenAngles(1).lower = 3.2;
agv(9).pivotParam(3).forbiddenAngles(2).higher = -3.2;
agv(9).pivotParam(3).forbiddenAngles(2).lower = -2.2;

agv(9).pivotValues(1).angularPosition = 0;
agv(9).pivotValues(1).velocity = 0;
agv(9).pivotValues(2).angularPosition = 0;
agv(9).pivotValues(2).velocity = 0;
agv(9).pivotValues(3).angularPosition = 0;
agv(9).pivotValues(3).velocity = 0;

agv(9).agvMeasures.front = 1; % in [m]
agv(9).agvMeasures.back  = 0.5; % in [m]
agv(9).agvMeasures.left  = 0.5; % in [m]
agv(9).agvMeasures.right = 0.5; % in [m]

%     sensor.type = 'ANS';
%     sensor.ans.targetDistance = 0.5; % in [m]
agv(9).sensor.type = 'Y';
agv(9).sensor.pos(1) = 0.5;
agv(9).sensor.pos(2) = 0.0;
agv(9).sensor.range = 0.8;

agv(9).velocity.vMax.x = 2; % [m/s]
agv(9).velocity.vMax.y = 0.7; % [m/s]
agv(9).velocity.vMax.a = 2; % [rad/s]
agv(9).velocity.vMax.xy = 3;

agv(9).velocity.acc.x = 1; % [m/s²]
agv(9).velocity.acc.y = 0.5; % [m/s²]
agv(9).velocity.acc.a = 3; % [rad/s²]
agv(9).velocity.acc.xy = 3;

agv(9).velocity.dec.x = 0.8;  % [m/s²]
agv(9).velocity.dec.y = 0.8;  % [m/s²]
agv(9).velocity.dec.a = 4;  % [rad/s²]
agv(9).velocity.dec.xy = 2;

agv(9).velocity.trackSpeed = 0.5;

%% 'BMF_TricycleRear'
agv(10).name='BMF_TricycleRear';
%      motorM240 = tf(1);
%     [num,den] = tfdata(motorM240,'v');
agv(10).motor = 1;

agv(10).n = 2;
agv(10).type = 'TricycleRear';
agv(10).pivotParam(1).position.x = 2.1115;
agv(10).pivotParam(1).position.y = 0.0;
agv(10).pivotParam(1).diameter = 0.2;
agv(10).pivotParam(1).gearratio = 1;
agv(10).pivotParam(1).forbiddenAngles(1).higher = 2.2;
agv(10).pivotParam(1).forbiddenAngles(1).lower = 3.2;
agv(10).pivotParam(1).forbiddenAngles(2).higher = -3.2;
agv(10).pivotParam(1).forbiddenAngles(2).lower = -2.2;

agv(10).pivotParam(2).diameter = 0.2;
agv(10).pivotParam(2).gearratio = 1;
agv(10).pivotParam(2).ringmount = 0.802;

agv(10).pivotValues(1).angularPosition = 0;
agv(10).pivotValues(1).velocity = 0;
agv(10).pivotValues(2).angularPosition = 0;
agv(10).pivotValues(2).velocity = 0;

agv(10).agvMeasures.front = 2.5; % in [m]
agv(10).agvMeasures.back  = 0.5; % in [m]
agv(10).agvMeasures.left  = 0.8; % in [m]
agv(10).agvMeasures.right = 0.8; % in [m]

agv(10).sensor.type = 'Y';
agv(10).sensor.pos(1) = 1.037;
agv(10).sensor.pos(2) = 0.0;
agv(10).sensor.range = 0.7; % PXV :: +/-35mm

agv(10).control.pTrack = 0.5;
agv(10).control.iTrack = 0.0;
agv(10).control.pivot.omegaMaxRadS = 2*2.11865346 / 180 * pi; % Geschwindigkeit berechnet von Wolfgang Umhau
agv(10).control.pivot.maxAccelerationWheel = 4;
agv(10).control.pivot.kp = 0.002;
agv(10).control.pivot.angularTolerance = pi; % in [rad] -- no initial steering, to avoid unnecessary stand rotations

agv(10).velocity.vsmax = 0.1; % in [m/s]
agv(10).velocity.trackSpeed = 0.1;

agv(10).velocity.vMax.x = 2; % [m/s]
agv(10).velocity.vMax.y = 0.7; % [m/s]
agv(10).velocity.vMax.a = 2; % [rad/s]
agv(10).velocity.vMax.xy = 3;

agv(10).velocity.acc.x = 1; % [m/s²]
agv(10).velocity.acc.y = 0.5; % [m/s²]
agv(10).velocity.acc.a = 3; % [rad/s²]
agv(10).velocity.acc.xy = 3;

agv(10).velocity.dec.x = 0.8;  % [m/s²]
agv(10).velocity.dec.y = 0.8;  % [m/s²]
agv(10).velocity.dec.a = 4;  % [rad/s²]
agv(10).velocity.dec.xy = 2;

%% 'BMF_SteeredDiff_SteerMaster'
agv(11).name='BMF_SteeredDiff_SteerMaster';
% motorM240 = tf(1);
%     [num,den] = tfdata(motorM240,'v');
agv(11).motor = 1;

agv(11).n = 2;
agv(11).type = 'SteeredDiff_SteerMaster';
agv(11).pivotParam(1).position.x = 2.1115;
agv(11).pivotParam(1).position.y = 0.0;
agv(11).pivotParam(1).diameter = 0.2;
agv(11).pivotParam(1).gearratio = 1;
agv(11).pivotParam(1).forbiddenAngles(1).higher = 2.2;
agv(11).pivotParam(1).forbiddenAngles(1).lower = 3.2;
agv(11).pivotParam(1).forbiddenAngles(2).higher = -3.2;
agv(11).pivotParam(1).forbiddenAngles(2).lower = -2.2;

agv(11).pivotParam(2).diameter = 0.2;
agv(11).pivotParam(2).gearratio = 1;
agv(11).pivotParam(2).ringmount = 0.802;

agv(11).pivotValues(1).angularPosition = 0;
agv(11).pivotValues(1).velocity = 0;
agv(11).pivotValues(2).angularPosition = 0;
agv(11).pivotValues(2).velocity = 0;

agv(11).agvMeasures.front = 2.5; % in [m]
agv(11).agvMeasures.back  = 0.5; % in [m]
agv(11).agvMeasures.left  = 0.8; % in [m]
agv(11).agvMeasures.right = 0.8; % in [m]

agv(11).sensor.type = 'Y';
agv(11).sensor.pos(1) = 0.3;
agv(11).sensor.pos(2) = 0.0;
agv(11).sensor.range = 0.4;

agv(11).velocity.vMax.x = 2; % [m/s]
agv(11).velocity.vMax.y = 0.7; % [m/s]
agv(11).velocity.vMax.a = 2; % [rad/s]
agv(11).velocity.vMax.xy = 3;

agv(11).velocity.acc.x = 1; % [m/s²]
agv(11).velocity.acc.y = 0.5; % [m/s²]
agv(11).velocity.acc.a = 3; % [rad/s²]
agv(11).velocity.acc.xy = 3;

agv(11).velocity.dec.x = 0.8;  % [m/s²]
agv(11).velocity.dec.y = 0.8;  % [m/s²]
agv(11).velocity.dec.a = 4;  % [rad/s²]
agv(11).velocity.dec.xy = 2;

%% 'BMF_SteeredDiff_DiffMaster'

agv(12).name='BMF_SteeredDiff_DiffMaster';
% motorM240 = tf(1);
%     [num,den] = tfdata(motorM240,'v');
agv(12).motor = 1;

agv(12).n = 2;
agv(12).type = 'SteeredDiff_DiffMaster';
agv(12).pivotParam(1).position.x = 2.1115;
agv(12).pivotParam(1).position.y = 0.0;
agv(12).pivotParam(1).diameter = 0.2;
agv(12).pivotParam(1).gearratio = 1;
agv(12).pivotParam(1).forbiddenAngles(1).higher = 2.2;
agv(12).pivotParam(1).forbiddenAngles(1).lower = 3.2;
agv(12).pivotParam(1).forbiddenAngles(2).higher = -3.2;
agv(12).pivotParam(1).forbiddenAngles(2).lower = -2.2;

agv(12).pivotParam(2).diameter = 0.2;
agv(12).pivotParam(2).gearratio = 1;
agv(12).pivotParam(2).ringmount = 0.802;

agv(12).pivotValues(1).angularPosition = 0;
agv(12).pivotValues(1).velocity = 0;
agv(12).pivotValues(2).angularPosition = 0;
agv(12).pivotValues(2).velocity = 0;

agv(12).agvMeasures.front = 2.5; % in [m]
agv(12).agvMeasures.back  = 0.5; % in [m]
agv(12).agvMeasures.left  = 0.8; % in [m]
agv(12).agvMeasures.right = 0.8; % in [m]

agv(12).sensor.type = 'Y';
agv(12).sensor.pos(1) = 1.037;
agv(12).sensor.pos(2) = 0.0;
agv(12).sensor.range = 0.4;

agv(12).velocity.vMax.x = 2; % [m/s]
agv(12).velocity.vMax.y = 0.7; % [m/s]
agv(12).velocity.vMax.a = 2; % [rad/s]
agv(12).velocity.vMax.xy = 3;

agv(12).velocity.acc.x = 1; % [m/s²]
agv(12).velocity.acc.y = 0.5; % [m/s²]
agv(12).velocity.acc.a = 3; % [rad/s²]
agv(12).velocity.acc.xy = 3;

agv(12).velocity.dec.x = 0.8;  % [m/s²]
agv(12).velocity.dec.y = 0.8;  % [m/s²]
agv(12).velocity.dec.a = 4;  % [rad/s²]
agv(12).velocity.dec.xy = 2;

%% 'Baer'
agv(13).name='Baer';
%  motorM240 = tf(1);
%     [num,den] = tfdata(motorM240,'v');
agv(13).motor = 1;

agv(13).n = 2;
agv(13).type = 'PivotActive';
agv(13).pivotParam(1).position.x = 0.7;
agv(13).pivotParam(1).position.y = 0.2;
agv(13).pivotParam(1).ringmount = 0;
agv(13).pivotParam(1).diameter = 0.2;
agv(13).pivotParam(1).gearratio = 1;
agv(13).pivotParam(1).forbiddenAngles(1).higher = 181;
agv(13).pivotParam(1).forbiddenAngles(1).lower = 120;
agv(13).pivotParam(1).forbiddenAngles(2).higher = -120;
agv(13).pivotParam(1).forbiddenAngles(2).lower = -181;

agv(13).pivotParam(2).position.x = -0.7;
agv(13).pivotParam(2).position.y = -0.2;
agv(13).pivotParam(2).ringmount = 0;
agv(13).pivotParam(2).diameter = 0.2;
agv(13).pivotParam(2).gearratio = 1;
agv(13).pivotParam(2).forbiddenAngles(1).higher = 181;
agv(13).pivotParam(2).forbiddenAngles(1).lower = 120;
agv(13).pivotParam(2).forbiddenAngles(2).higher = -120;
agv(13).pivotParam(2).forbiddenAngles(2).lower = -181;

agv(13).pivotValues(1).angularPosition = 0;
agv(13).pivotValues(1).velocity = 0;
agv(13).pivotValues(2).angularPosition = 0;
agv(13).pivotValues(2).velocity = 0;

agv(13).agvMeasures.front = 1; % in [m]
agv(13).agvMeasures.back  = 1; % in [m]
agv(13).agvMeasures.left  = 0.5; % in [m]
agv(13).agvMeasures.right = 0.5; % in [m]

agv(13).sensor.type = 'YA';
agv(13).sensor.pos(1) = 0.0;
agv(13).sensor.pos(2) = 0.1;
agv(13).sensor.range = 0.6;

agv(13).velocity.vMax.x = 2; % [m/s]
agv(13).velocity.vMax.y = 0.7; % [m/s]
agv(13).velocity.vMax.a = 2; % [rad/s]
agv(13).velocity.vMax.xy = 3;

agv(13).velocity.acc.x = 1; % [m/s²]
agv(13).velocity.acc.y = 0.5; % [m/s²]
agv(13).velocity.acc.a = 3; % [rad/s²]
agv(13).velocity.acc.xy = 3;

agv(13).velocity.dec.x = 0.8;  % [m/s²]
agv(13).velocity.dec.y = 0.8;  % [m/s²]
agv(13).velocity.dec.a = 4;  % [rad/s²]
agv(13).velocity.dec.xy = 2;

%% 'VDL'
agv(14).name='VDL';
% motorM240 = tf(1);
%     [num,den] = tfdata(motorM240,'v');
agv(14).motor = 1;

agv(14).n = 3;
agv(14).type = 'PivotActive';
agv(14).pivotParam(1).position.x = 0.7;
agv(14).pivotParam(1).position.y = 0;
agv(14).pivotParam(1).ringmount = 0;
agv(14).pivotParam(1).diameter = 0.2;
agv(14).pivotParam(1).gearratio = 1;
agv(14).pivotParam(1).forbiddenAngles(1).higher = 181;
agv(14).pivotParam(1).forbiddenAngles(1).lower = 120;
agv(14).pivotParam(1).forbiddenAngles(2).higher = -120;
agv(14).pivotParam(1).forbiddenAngles(2).lower = -181;

agv(14).pivotParam(2).position.x = -0.4;
agv(14).pivotParam(2).position.y = 0;
agv(14).pivotParam(2).ringmount = 0;
agv(14).pivotParam(2).diameter = 0.2;
agv(14).pivotParam(2).gearratio = 1;
agv(14).pivotParam(2).forbiddenAngles(1).higher = 181;
agv(14).pivotParam(2).forbiddenAngles(1).lower = 120;
agv(14).pivotParam(2).forbiddenAngles(2).higher = -120;
agv(14).pivotParam(2).forbiddenAngles(2).lower = -181;

agv(14).pivotParam(3).position.x = 0.2;%0;
agv(14).pivotParam(3).position.y = -0.4;%0;
agv(14).pivotParam(3).ringmount = 0;
agv(14).pivotParam(3).diameter = 0.2;
agv(14).pivotParam(3).gearratio = 1;
agv(14).pivotParam(3).forbiddenAngles(1).higher = 181;
agv(14).pivotParam(3).forbiddenAngles(1).lower = 120;
agv(14).pivotParam(3).forbiddenAngles(2).higher = -120;
agv(14).pivotParam(3).forbiddenAngles(2).lower = -181;

agv(14).pivotValues(1).angularPosition = 0;
agv(14).pivotValues(1).velocity = 0;
agv(14).pivotValues(2).angularPosition = 0;
agv(14).pivotValues(2).velocity = 0;
agv(14).pivotValues(3).angularPosition = 0;
agv(14).pivotValues(3).velocity = 0;

agv(14).agvMeasures.front = 1; % in [m]
agv(14).agvMeasures.back  = 0.6; % in [m]
agv(14).agvMeasures.left  = 0.5; % in [m]
agv(14).agvMeasures.right = 0.5; % in [m]

agv(14).sensor.type = 'ANS';
agv(14).sensor.ans.targetDistance = 0.5; % in [m]

agv(14).velocity.vMax.x = 2; % [m/s]
agv(14).velocity.vMax.y = 0.7; % [m/s]
agv(14).velocity.vMax.a = 2; % [rad/s]
agv(14).velocity.vMax.xy = 3;

agv(14).velocity.acc.x = 1; % [m/s²]
agv(14).velocity.acc.y = 0.5; % [m/s²]
agv(14).velocity.acc.a = 3; % [rad/s²]
agv(14).velocity.acc.xy = 3;

agv(14).velocity.dec.x = 0.8;  % [m/s²]
agv(14).velocity.dec.y = 0.8;  % [m/s²]
agv(14).velocity.dec.a = 4;  % [rad/s²]
agv(14).velocity.dec.xy = 2;

%% 'Evocortex'

agv(15).name='Evocortex';
agv(15).motor = 1;

agv(15).n = 4;
agv(15).type = 'Mecanum';
agv(15).pivotParam(1).position.x = 0.8;
agv(15).pivotParam(1).position.y = 0.5;
agv(15).pivotParam(1).ringmount = 0;
agv(15).pivotParam(1).diameter = 0.2;
agv(15).pivotParam(1).gearratio = 1;

agv(15).pivotParam(2) = agv(15).pivotParam(1);
agv(15).pivotParam(2).position.x = 0.8;
agv(15).pivotParam(2).position.y = -0.5;

agv(15).pivotParam(3) = agv(15).pivotParam(2);
agv(15).pivotParam(3).position.x = -0.8;
agv(15).pivotParam(3).position.y = 0.5;

agv(15).pivotParam(4) = agv(15).pivotParam(3);
agv(15).pivotParam(4).position.x = -0.8;
agv(15).pivotParam(4).position.y = -0.5;

agv(15).agvMeasures.front = 1; % in [m]
agv(15).agvMeasures.back  = 1; % in [m]
agv(15).agvMeasures.left  = 0.5; % in [m]
agv(15).agvMeasures.right = 0.5; % in [m]

agv(15).sensor.type = 'ANS';
agv(15).sensor.ans.targetDistance = 1; % in [m]
agv(15).pivotValues = 0;

agv(15).velocity.vMax.x = 2; % [m/s]
agv(15).velocity.vMax.y = 0.7; % [m/s]
agv(15).velocity.vMax.a = 2; % [rad/s]
agv(15).velocity.vMax.xy = 3;

agv(15).velocity.acc.x = 1; % [m/s²]
agv(15).velocity.acc.y = 0.5; % [m/s²]
agv(15).velocity.acc.a = 3; % [rad/s²]
agv(15).velocity.acc.xy = 3;

agv(15).velocity.dec.x = 0.8;  % [m/s²]
agv(15).velocity.dec.y = 0.8;  % [m/s²]
agv(15).velocity.dec.a = 4;  % [rad/s²]
agv(15).velocity.dec.xy = 2;

%% 'Mecanum_Test'
agv(16).name='Mecanum_Test';
agv(16).motor = 1;

agv(16).n = 4;
agv(16).type = 'Mecanum';

agv(16).pivotParam(1).position.x = 0.2;
agv(16).pivotParam(1).position.y = 0.2;
agv(16).pivotParam(1).ringmount = 0;
agv(16).pivotParam(1).diameter = 0.2;
agv(16).pivotParam(1).gearratio = 1;

agv(16).pivotParam(2) = agv(16).pivotParam(1);
agv(16).pivotParam(2).position.x = 0.2;
agv(16).pivotParam(2).position.y = -0.2;

agv(16).pivotParam(3) = agv(16).pivotParam(2);
agv(16).pivotParam(3).position.x = -0.2;
agv(16).pivotParam(3).position.y = 0.2;

agv(16).pivotParam(4) = agv(16).pivotParam(3);
agv(16).pivotParam(4).position.x = -0.2;
agv(16).pivotParam(4).position.y = -0.2;

agv(16).agvMeasures.front = 0.3; % in [m]
agv(16).agvMeasures.back  = 0.3; % in [m]
agv(16).agvMeasures.left  = 0.3; % in [m]
agv(16).agvMeasures.right = 0.3; % in [m]

agv(16).sensor.type = 'YA';
agv(16).sensor.pos(1) = 0.5;
agv(16).sensor.pos(2) = 0.0;
agv(16).sensor.range = 0.6;

agv(16).pivotValues = 0;

agv(16).velocity.trackSpeed = 0.3;

agv(16).velocity.vMax.x = 2; % [m/s]
agv(16).velocity.vMax.y = 0.7; % [m/s]
agv(16).velocity.vMax.a = 2; % [rad/s]
agv(16).velocity.vMax.xy = 3;

agv(16).velocity.acc.x = 1; % [m/s²]
agv(16).velocity.acc.y = 0.5; % [m/s²]
agv(16).velocity.acc.a = 3; % [rad/s²]
agv(16).velocity.acc.xy = 3;

agv(16).velocity.dec.x = 0.8;  % [m/s²]
agv(16).velocity.dec.y = 0.8;  % [m/s²]
agv(16).velocity.dec.a = 4;  % [rad/s²]
agv(16).velocity.dec.xy = 2;

agv(16).control.y.pTrack = 0.2;
agv(16).control.y.iTrack = 0.0;
agv(16).control.a.pTrack = 0.5;
agv(16).control.a.iTrack = 0.0;

%     control.pivot.omegaMaxRadS = 0.5;
%     control.pivot.maxAccelerationWheel = 4;
%     control.pivot.kp = 0.02;
%     control.pivot.angularTolerance = pi; % in [rad] -- no initial steering, to avoid unnecessary stand rotations


%% 'Eckhart'

agv(17).name='Eckhart';

agv(17).motor = 1;

agv(17).n = 2;
agv(17).type = 'SteeredDiff_OneWheel';
agv(17).pivotParam(1).position.x = 0.8;
agv(17).pivotParam(1).position.y = 0.0;
agv(17).pivotParam(1).ringmount = 0;
agv(17).pivotParam(1).diameter = 0.2;
agv(17).pivotParam(1).gearratio = 1;
agv(17).pivotParam(1).forbiddenAngles(1).higher = 181;
agv(17).pivotParam(1).forbiddenAngles(1).lower = 120;
agv(17).pivotParam(1).forbiddenAngles(2).higher = -120;
agv(17).pivotParam(1).forbiddenAngles(2).lower = -181;

agv(17).pivotParam(2) = agv(17).pivotParam(1);
agv(17).pivotParam(2).position.x = 0.0;
agv(17).pivotParam(2).ringmount = 0.5;

agv(17).agvMeasures.front = 1; % in [m]
agv(17).agvMeasures.back  = 0.2; % in [m]
agv(17).agvMeasures.left  = 0.5; % in [m]
agv(17).agvMeasures.right = 0.5; % in [m]


agv(17).sensor.type = 'Y';
agv(17).sensor.pos(1) = 0.5;
agv(17).sensor.pos(2) = 0.0;
agv(17).sensor.range = 0.4;

agv(17).velocity.trackSpeed = 0.2; % [m/s]
agv(17).velocity.vsmax = 0.2;

agv(17).velocity.vMax.x = 2; % [m/s]
agv(17).velocity.vMax.y = 0.7; % [m/s]
agv(17).velocity.vMax.a = 2; % [rad/s]
agv(17).velocity.vMax.xy = 3;

agv(17).velocity.acc.x = 1; % [m/s²]
agv(17).velocity.acc.y = 0.5; % [m/s²]
agv(17).velocity.acc.a = 3; % [rad/s²]
agv(17).velocity.acc.xy = 3;

agv(17).velocity.dec.x = 0.8;  % [m/s²]
agv(17).velocity.dec.y = 0.8;  % [m/s²]
agv(17).velocity.dec.a = 4;  % [rad/s²]
agv(17).velocity.dec.xy = 2;

agv(17).pivotValues(1).angularPosition = 0;
agv(17).pivotValues(1).velocity = 0;
agv(17).pivotValues(2) = agv(17).pivotValues(1);

agv(17).control.y.pTrack = 1;
agv(17).control.y.iTrack = 0.0;
agv(17).control.pivot.omegaMaxRadS = 0.5;
agv(17).control.pivot.maxAccelerationWheel = 4;
agv(17).control.pivot.kp = 0.02;
agv(17).control.pivot.angularTolerance = pi; % in [rad] -- no initial steering, to avoid unnecessary stand rotations


%% 'K55'
agv(18).name='K55';
%     load motorWFT.mat tfunction
%     motorWFT = c2d(tfunction,sampleTime);
% %     motorWFT = c2d(tf(1),sampleTime);
%     [num,den] = tfdata(motorWFT,'v');
agv(18).motor = [ 0   -0.0001    0.0002; ...
    1.0000   -1.9927    0.9928];

agv(18).n = 1;
agv(18).type = 'Tricycle';
agv(18).agvMeasures.front = 1.0914 + 0.2935; % in [m]
agv(18).agvMeasures.back  = 1.8989 - 0.2935 - 1.0914; % in [m]
agv(18).agvMeasures.left  = 0.903 / 2; % in [m]
agv(18).agvMeasures.right = 0.903 / 2; % in [m]
agv(18).pivotParam(1).position.x = 1.0914;  % in [m]
agv(18).pivotParam(1).position.y = 0.0;  % in [m]
agv(18).pivotParam(1).ringmount = 0.2; % in [m]
agv(18).pivotParam(1).diameter = 0.2;  % in [m]
agv(18).pivotParam(1).gearratio = 1;

agv(18).pivotValues(1).angularPosition = 0;
agv(18).pivotValues(1).velocity = 0;

agv(18).sensor.type = 'ANS';
agv(18).sensor.pos(1) = 0.5;
agv(18).sensor.pos(2) = 0.0;
agv(18).sensor.range = 1;

agv(18).sensor.ans.targetDistance = 1.5; % in [m]
agv(18).sensor.ans.targetXValue = -1; % in [m] -- "-1" is astolfi controller

agv(18).velocity.trackSpeed = 0.5; % [m/s]

agv(18).velocity.vMax.x = 2; % [m/s]
agv(18).velocity.vMax.y = 0.7; % [m/s]
agv(18).velocity.vMax.a = 2; % [rad/s]
agv(18).velocity.vMax.xy = 3;

agv(18).velocity.acc.x = 1; % [m/s²]
agv(18).velocity.acc.y = 0.5; % [m/s²]
agv(18).velocity.acc.a = 3; % [rad/s²]
agv(18).velocity.acc.xy = 3;

agv(18).velocity.dec.x = 0.8;  % [m/s²]
agv(18).velocity.dec.y = 0.8;  % [m/s²]
agv(18).velocity.dec.a = 4;  % [rad/s²]
agv(18).velocity.dec.xy = 2;

agv(18).control.pivot.omegaMaxRadS = 0.5;
agv(18).control.pivot.maxAccelerationWheel = 4;
agv(18).control.pivot.kp = 0.2;
agv(18).control.pivot.angularTolerance = pi; % in [rad] -- no initial steering, to avoid unnecessary stand rotations

agv(18).control.y.pTrack = 2.5;
agv(18).control.y.iTrack = 0.0;
agv(18).control.y.dTrack = 0.0;

agv(18).control.a.pTrack = 0;
agv(18).control.a.iTrack = 0;
agv(18).control.a.dTrack = 0;


%% 'normDiffANS'

agv(19).name='normDiffANS';
%     load motorWFT.mat tfunction
%     motorWFT = c2d(tfunction,sampleTime);
%     [num,den] = tfdata(motorWFT,'v');
agv(19).motor = [ 0   -0.0001    0.0002; ...
    1.0000   -1.9927    0.9928];

agv(19).n = 1;
agv(19).type = 'Diff';
agv(19).agvMeasures.front = 1; % in [m]
agv(19).agvMeasures.back  = 0.5; % in [m]
agv(19).agvMeasures.left  = 0.4; % in [m]
agv(19).agvMeasures.right = 0.4; % in [m]
agv(19).pivotParam(1).position.x = 0;  % in [m]
agv(19).pivotParam(1).position.y = 0;  % in [m]
agv(19).pivotParam(1).ringmount = 0.2; % in [m]
agv(19).pivotParam(1).diameter = 0.2;  % in [m]
agv(19).pivotParam(1).gearratio = 1;

agv(19).pivotValues(1).angularPosition = 0;
agv(19).pivotValues(1).velocity = 0;

agv(19).sensor.type = 'ANS';

agv(19).sensor.ans.targetDistance = 1.5; % in [m]
agv(19).sensor.ans.targetXValue = -1; % in [m] -- "-1" is astolfi controller

agv(19).velocity.trackSpeed = 0.5; % [m/s]



%% 'normDiffPGV'
agv(20).name='normDiffPGV';

% load motorWFT.mat tfunction
%     motorWFT = c2d(tfunction,sampleTime);
%     [num,den] = tfdata(motorWFT,'v');
agv(20).motor = [ 0   -0.0001    0.0002; ...
    1.0000   -1.9927    0.9928];


agv(20).n = 1;
agv(20).type = 'Diff';
agv(20).agvMeasures.front = 1; % in [m]
agv(20).agvMeasures.back  = 0.5; % in [m]
agv(20).agvMeasures.left  = 0.4; % in [m]
agv(20).agvMeasures.right = 0.4; % in [m]
agv(20).pivotParam(1).position.x = 0;  % in [m]
agv(20).pivotParam(1).position.y = 0;  % in [m]
agv(20).pivotParam(1).ringmount = 0.2; % in [m]
agv(20).pivotParam(1).diameter = 0.2;  % in [m]
agv(20).pivotParam(1).gearratio = 1;

agv(20).pivotValues(1).angularPosition = 0;
agv(20).pivotValues(1).velocity = 0;

agv(20).sensor.type = 'YA';
agv(20).sensor.pos(1) = 0.5;
agv(20).sensor.pos(2) = 0.0;
agv(20).sensor.range = 0.5;

agv(20).velocity.trackSpeed = 0.5; % [m/s]



%% 'normTriANS'
agv(21).name='normTriANS';
%     load motorWFT.mat tfunction
%     motorWFT = c2d(tfunction,sampleTime);
%     [num,den] = tfdata(motorWFT,'v');
agv(21).motor = [ 0   -0.0001    0.0002; ...
    1.0000   -1.9927    0.9928];

agv(21).n = 1;
agv(21).type = 'Tricycle';
agv(21).agvMeasures.front = 1; % in [m]
agv(21).agvMeasures.back  = 0.5; % in [m]
agv(21).agvMeasures.left  = 0.4; % in [m]
agv(21).agvMeasures.right = 0.4; % in [m]
agv(21).pivotParam(1).position.x = 0.75;  % in [m]
agv(21).pivotParam(1).position.y = 0;  % in [m]
agv(21).pivotParam(1).ringmount = 0.2; % in [m]
agv(21).pivotParam(1).diameter = 0.2;  % in [m]
agv(21).pivotParam(1).gearratio = 1;

agv(21).pivotValues(1).angularPosition = 0;
agv(21).pivotValues(1).velocity = 0;

agv(21).sensor.type = 'ANS';

agv(21).sensor.ans.targetDistance = 1.5; % in [m]
agv(21).sensor.ans.targetXValue = -1; % in [m] -- "-1" is astolfi controller

agv(21).velocity.trackSpeed = 0.5; % [m/s]

agv(21).control.pivot.kp = 0.01;


%% 'normTriPGV'
agv(22).name='normTriPGV';
% load motorWFT.mat tfunction
%     motorWFT = c2d(tfunction,sampleTime);
%     [num,den] = tfdata(motorWFT,'v');
agv(22).motor = [ 0   -0.0001    0.0002; ...
    1.0000   -1.9927    0.9928];

agv(22).n = 1;
agv(22).type = 'Tricycle';
agv(22).agvMeasures.front = 1; % in [m]
agv(22).agvMeasures.back  = 0.5; % in [m]
agv(22).agvMeasures.left  = 0.4; % in [m]
agv(22).agvMeasures.right = 0.4; % in [m]
agv(22).pivotParam(1).position.x = 0.75;  % in [m]
agv(22).pivotParam(1).position.y = 0;  % in [m]
agv(22).pivotParam(1).ringmount = 0.2; % in [m]
agv(22).pivotParam(1).diameter = 0.2;  % in [m]
agv(22).pivotParam(1).gearratio = 1;

agv(22).pivotValues(1).angularPosition = 0;
agv(22).pivotValues(1).velocity = 0;

agv(22).sensor.type = 'YA';
agv(22).sensor.pos(1) = 0.5;
agv(22).sensor.pos(2) = 0.0;
agv(22).sensor.range = 0.5;

agv(22).velocity.trackSpeed = 0.5; % [m/s]

agv(22).control.pivot.kp = 0.01; % 0.05 ist instabil auf sCurve

%% 'normNpiANS'

agv(23).name='normNpiANS';
%     load motorWFT.mat tfunction
%     motorWFT = c2d(tfunction,sampleTime);
%     [num,den] = tfdata(motorWFT,'v');
agv(23).motor =[ 0   -0.0001    0.0002; ...
    1.0000   -1.9927    0.9928];

agv(23).agvMeasures.front = 0.75; % in [m]
agv(23).agvMeasures.back  = 0.75; % in [m]
agv(23).agvMeasures.left  = 0.4; % in [m]
agv(23).agvMeasures.right = 0.4; % in [m]

agv(23).n = 2;
agv(23).type = 'PivotPassive';
agv(23).pivotParam(1).position.x = 0.5;
agv(23).pivotParam(1).position.y = 0.2;
agv(23).pivotParam(1).ringmount = 0.1;
agv(23).pivotParam(1).diameter = 0.2;
agv(23).pivotParam(1).gearratio = 1;
agv(23).pivotParam(1).forbiddenAngles(1).higher = 181;
agv(23).pivotParam(1).forbiddenAngles(1).lower = 120;
agv(23).pivotParam(1).forbiddenAngles(2).higher = -120;
agv(23).pivotParam(1).forbiddenAngles(2).lower = -181;

agv(23).pivotParam(2).position.x = -0.5;
agv(23).pivotParam(2).position.y = -0.2;
agv(23).pivotParam(2).ringmount = 0.1;
agv(23).pivotParam(2).diameter = 0.2;
agv(23).pivotParam(2).gearratio = 1;
agv(23).pivotParam(2).forbiddenAngles(1).higher = 181;
agv(23).pivotParam(2).forbiddenAngles(1).lower = 120;
agv(23).pivotParam(2).forbiddenAngles(2).higher = -120;
agv(23).pivotParam(2).forbiddenAngles(2).lower = -181;

agv(23).pivotValues(1).angularPosition = 0;
agv(23).pivotValues(1).velocity = 0;
agv(23).pivotValues(2).angularPosition = 0;
agv(23).pivotValues(2).velocity = 0;

agv(23).sensor.type = 'ANS';
agv(23).sensor.ans.targetDistance = 1.5; % in [m]
agv(23).sensor.ans.targetXValue = -1; % in [m] -- "-1" is astolfi controller

agv(23).velocity.trackSpeed = 0.5; % [m/s]

agv(23).control.pivot.kp = 0.01;


%% 'normNpiPGV'
agv(24).name='normNpiPGV';
% load motorWFT.mat tfunction
%     motorWFT = c2d(tfunction,sampleTime);
%     [num,den] = tfdata(motorWFT,'v');
agv(24).motor = [ 0   -0.0001    0.0002; ...
    1.0000   -1.9927    0.9928];


agv(24).agvMeasures.front = 0.75; % in [m]
agv(24).agvMeasures.back  = 0.75; % in [m]
agv(24).agvMeasures.left  = 0.4; % in [m]
agv(24).agvMeasures.right = 0.4; % in [m]

agv(24).n = 2;
agv(24).type = 'PivotPassive';
agv(24).pivotParam(1).position.x = 0.5;
agv(24).pivotParam(1).position.y = 0.2;
agv(24).pivotParam(1).ringmount = 0.1;
agv(24).pivotParam(1).diameter = 0.2;
agv(24).pivotParam(1).gearratio = 1;
agv(24).pivotParam(1).forbiddenAngles(1).higher = 181;
agv(24).pivotParam(1).forbiddenAngles(1).lower = 120;
agv(24).pivotParam(1).forbiddenAngles(2).higher = -120;
agv(24).pivotParam(1).forbiddenAngles(2).lower = -181;

agv(24).pivotParam(2).position.x = -0.5;
agv(24).pivotParam(2).position.y = -0.2;
agv(24).pivotParam(2).ringmount = 0.1;
agv(24).pivotParam(2).diameter = 0.2;
agv(24).pivotParam(2).gearratio = 1;
agv(24).pivotParam(2).forbiddenAngles(1).higher = 181;
agv(24).pivotParam(2).forbiddenAngles(1).lower = 120;
agv(24).pivotParam(2).forbiddenAngles(2).higher = -120;
agv(24).pivotParam(2).forbiddenAngles(2).lower = -181;

agv(24).pivotValues(1).angularPosition = 0;
agv(24).pivotValues(1).velocity = 0;
agv(24).pivotValues(2).angularPosition = 0;
agv(24).pivotValues(2).velocity = 0;

agv(24).sensor.type = 'YA';
agv(24).sensor.pos(1) = 0.5;
agv(24).sensor.pos(2) = 0.0;
agv(24).sensor.range = 0.5;

agv(24).velocity.trackSpeed = 0.5; % [m/s]

agv(24).control.pivot.kp = 0.01;




end