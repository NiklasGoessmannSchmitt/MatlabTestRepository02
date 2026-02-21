% close all
clear all
clc

%% To Do
% check why this is wrong 
% figure
% plot(a.betaSetpointBuffer')
% grid on 
% hold on 
% plot(a.betaCurrentBuffer')

%%
curdir=pwd;
folderstrct = strfind(curdir,'\');
newdir = curdir(1:folderstrct(end)-1);
addpath(genpath(newdir))

%% AGV setup
samplingTime = 1e-3;
[pivotParam, agvType, motor, n, agvMeasures, pivotValues, sensor, name, control, velocity] = AGVconfig('VDL', samplingTime); 

vX = zeros(1,10000);
vX(500:3000) = 1; 
vY = zeros(1,length(vX)); 
vY(3000:end) = 0.2; 
vA = zeros(1,length(vX)); 
vA(1500:end) = 0.1;
vSet = [vX; vY; vA]; 

g = initGeneralValues(0, 0, 0, length(vSet));
a = initAgvValues(g, n, agvType, motor, agvMeasures, sensor, pivotParam, pivotValues);
ideal = a; 

v = [0, 0, 0];
vScaledOld = v;
minPercentage = 1; 

vMod = [0,0,0];
angularPosition = [0,0,0];
angularVelocity = [0,0,0];
possibleAngularDeviationPercentage = [0,0,0];

stopReq = false; 

for i = 1:length(vSet)

    %% Velocity Ramping
    [vRamped, ~, ~, accBit, decBit] = LSimoveC_UC_MoveCtrl_Ramp ...
        (stopReq, (~a.statSteering || a.statMovement), 1, vSet(:,i), velocity.vMax, velocity.acc, velocity.dec, samplingTime);
    
        vRampedBuffer(:,i) = vRamped; 

    %% Kinematics :: NPivotActive
    omegaMax = 2; % [rad/s]
    iTolerance = 4e-3;
    iStandstill = true; 
    
    [vs, a.betaSetpoints] = LSimoveC_UC_MoveCtrl_NPivot_V2W(vRamped, ...
                                                            a.pivotValues, ...
                                                            a.pivotParam, ...
                                                            n, ...
                                                            omegaMax, ...
                                                            a.rampingFactorSteering, ...
                                                            iTolerance, ...
                                                            0.5, ...
                                                            iStandstill, ...
                                                            0);
    a.betaSetpointBuffer(:,i) = a.betaSetpoints; 
    
    %% IDEAL   
    ideal.steeringVelocity(1,i) = (a.betaSetpoints(1) - ideal.pivotValues(1).angularPosition) / samplingTime;
    ideal.steeringVelocity(2,i) = (a.betaSetpoints(2) - ideal.pivotValues(2).angularPosition) / samplingTime;
    ideal.steeringVelocity(3,i) = (a.betaSetpoints(3) - ideal.pivotValues(3).angularPosition) / samplingTime;
    ideal.pivotValues(1).angularPosition = a.betaSetpoints(1); 
    ideal.pivotValues(2).angularPosition = a.betaSetpoints(2); 
    ideal.pivotValues(3).angularPosition = a.betaSetpoints(3); 
    ideal.pivotValues(1).velocity = vs(1); 
    ideal.pivotValues(2).velocity = vs(2); 
    ideal.pivotValues(3).velocity = vs(3); 
    
    epsilon = 0.15;
    [vIdeal, omegaMaxErrorIdeal] = LSimoveC_UC_MoveCtrl_NPivot_W2V(n, ideal.pivotValues, epsilon, a.pivotParam);
    g.vCurrentIdeal(:,i) = vIdeal; 
    g.omegaMaxErrorIdeal(i) = omegaMaxErrorIdeal;

    %% CURRENT APPROACH
    iOmegaMaxRadS = 1.4; % [rad/s]
    wheelAcceleration = 1; % [rad/s²]
    
    desiredAngularDelta = (a.betaSetpoints - angularPosition);
        desiredAngularDeltaBuffer(:,i) = desiredAngularDelta; 

    % the current control deviation would require the following velocity
    desiredVelo = desiredAngularDelta ./ samplingTime; 
        desiredVeloBuffer(:,i) = desiredVelo; 

    % which would require the following acceleration
    desiredAcc = (desiredVelo - angularVelocity) ./ samplingTime; 
        desiredAccBuffer(:,i) = desiredAcc; 

    % however, only this acceleration is possible
    possibleAcc = desiredAcc; 
    possibleAcc(possibleAcc < -wheelAcceleration) = -wheelAcceleration; 
    possibleAcc(possibleAcc >  wheelAcceleration) =  wheelAcceleration; 
        possibleAccBuffer(:,i) = possibleAcc; 

    % with this acceleration the following velocity is possible
    possibleVelo = angularVelocity + possibleAcc .* samplingTime;     
        possibleVeloBuffer1(:,i) = possibleVelo;

    % however, will be limited to the maximum angular velocity
    possibleVelo(possibleVelo < -iOmegaMaxRadS) = -iOmegaMaxRadS;
    possibleVelo(possibleVelo >  iOmegaMaxRadS) =  iOmegaMaxRadS;
        possibleVeloBuffer2(:,i) = possibleVelo;

%     % and the desired angular velocity 
%     tempAngularVeloSetpoint(tempWheelCounter) = possibleVelo(tempWheelCounter);
%     if abs(possibleVelo(tempWheelCounter)) > abs(desiredVelo(tempWheelCounter))
%         tempAngularVeloSetpoint(tempWheelCounter) = desiredVelo(tempWheelCounter); 
%     end

    % and it must be possible to reduce the speed in order to not
    % overshoot
    vmax = sqrt(2*abs(desiredAngularDelta)./wheelAcceleration);
    possibleVelo(possibleVelo < -vmax) = -vmax(possibleVelo < -vmax); 
    possibleVelo(possibleVelo >  vmax) =  vmax(possibleVelo >  vmax); 
        possibleVeloBuffer(:,i) = possibleVelo;

    % and respectively the following angle 
    possibleAngularDeviation = possibleVelo .* samplingTime; 
        possibleAngularDeviationBuffer(:,i) = possibleAngularDeviation;

    % this is so many percent of the desired angle delta
    % limited to 1 because it is not necessary to overshoot
    for b = 1:length(possibleAngularDeviation)
        if desiredAngularDelta(b) ~= 0 
            possibleAngularDeviationPercentage(b) = possibleAngularDeviation(b) ./ desiredAngularDelta(b);
        else
            possibleAngularDeviationPercentage(b) = 1;
        end
    end

    minPossibleAngularDeviationPercentage = min(possibleAngularDeviationPercentage);
    
        possibleAngularDeviationPercentageBuffer(:,i) = possibleAngularDeviationPercentage;

    cartVelocityScaled = (vRamped - vMod)*max(0.001, minPossibleAngularDeviationPercentage) + vMod; 

    [vsMod, betaSetpointsMod] = LSimoveC_UC_MoveCtrl_NPivot_V2W(cartVelocityScaled, ...
                                                            a.pivotValues, ...
                                                            a.pivotParam, ...
                                                            n, ...
                                                            omegaMax, ...
                                                            a.rampingFactorSteering, ...
                                                            iTolerance, ...
                                                            0.5, ...
                                                            iStandstill, ...
                                                            0);

    desiredAngularDeltaMod = (betaSetpointsMod - angularPosition);

    % the current control deviation would require the following velocity
    desiredVeloMod = desiredAngularDeltaMod ./ samplingTime; 

    % which would require the following acceleration
    desiredAccMod = (desiredVeloMod - angularVelocity) ./ samplingTime; 
    
    angularVelocity = desiredVeloMod;
    angularPosition = desiredVeloMod * samplingTime + angularPosition; 

    angularAccelerationBuffer(:,i) = desiredAccMod; 
    angularVelocityBuffer(:,i) = angularVelocity; 
    angularPositionBuffer(:,i) = angularPosition; 

    modPivotValues(1).angularPosition = angularPosition(1);
    modPivotValues(2).angularPosition = angularPosition(2);   
    modPivotValues(3).angularPosition = angularPosition(3);
    modPivotValues(1).velocity = vsMod(1);
    modPivotValues(2).velocity = vsMod(2);
    modPivotValues(3).velocity = vsMod(3);

    epsilon = 0.15;
    [vMod, omegaMaxErrorMod] = LSimoveC_UC_MoveCtrl_NPivot_W2V(n, modPivotValues, epsilon, a.pivotParam); 

    %%
    vModBuffer(:,i) = vMod; 
    omegaMaxBufferMod(i) = omegaMaxErrorMod; 

end

%%
figure
subplot(211)
plot(a.betaSetpointBuffer'); grid on; hold on
plot(angularPositionBuffer')

subplot(212)
plot(desiredAngularDeltaBuffer'); grid on 

%%
figure
subplot(211)
plot(desiredVeloBuffer'); grid on; hold on 
plot(possibleVeloBuffer')

subplot(212)
plot(desiredAccBuffer'); grid on; hold on
plot(possibleAccBuffer')

%%
figure
subplot(311)
plot(angularPositionBuffer'); grid on

subplot(312)
plot(angularVelocityBuffer'); grid on

subplot(313)
plot(diff(angularVeloSetpointBuffer(1,:))'); grid on; hold on
plot(diff(angularVeloSetpointBuffer(2,:))')
plot(diff(angularVeloSetpointBuffer(3,:))')

%% 
figure
subplot(131)
plot(desiredVeloBuffer'); grid on 
title('desired velos')

subplot(132)
plot(possibleVeloBuffer'); grid on
title('possible velos')

subplot(133)
plot(angularVeloSetpointBuffer'); grid on 
title('setpoint velos')

%% 
figure
subplot(321)
plot(vSet(1,:)); grid on; hold on
plot(vRampedBuffer(1,:))
% plot(g.vCurrentIdeal(1,:))
plot(vCurrent(1,:))
% plot(vScaledBuffer(1,:))

subplot(323)
plot(vSet(2,:)); grid on; hold on
plot(vRampedBuffer(2,:))
% plot(g.vCurrentIdeal(2,:))
plot(vCurrent(2,:))
% plot(vScaledBuffer(2,:))

subplot(325)
plot(vSet(3,:)); grid on; hold on
plot(vRampedBuffer(3,:))
% plot(g.vCurrentIdeal(3,:))
plot(vCurrent(3,:))
% plot(vScaledBuffer(3,:))

% figure
subplot(422)
plot(a.betaCurrentBuffer(1,:),'Color','r','LineWidth',2); grid on; hold on
plot(a.betaCurrentBuffer(2,:),'Color','b','LineWidth',2)
plot(a.betaCurrentBuffer(3,:),'Color',[0,114,0]./256,'LineWidth',2)
% plot(betaModBuffer(1,:),'Color','r','LineWidth',1)
% plot(a.betaSetpointBuffer(1,:),'Color','r','LineWidth',1)
% plot(a.betaSetpointBuffer(2,:),'Color','b','LineWidth',1)
% plot(a.betaSetpointBuffer(3,:),'Color',[0,114,0]./256,'LineWidth',1)
title('angle')

subplot(424)
plot(a.steeringVelocityBuffer(1,:),'Color','r','LineWidth',2); grid on; hold on
plot(a.steeringVelocityBuffer(2,:),'Color','b','LineWidth',2)
plot(a.steeringVelocityBuffer(3,:),'Color',[0,114,0]./256,'LineWidth',2)
% plot(ideal.steeringVelocity(1,:),'Color','r','LineWidth',1)
% plot(ideal.steeringVelocity(2,:),'Color','b','LineWidth',1)
% plot(ideal.steeringVelocity(3,:),'Color',[0,114,0]./256,'LineWidth',1)
title('angularVelocity')

subplot(426)
plot(diff(a.steeringVelocityBuffer(1,:)),'Color','r','LineWidth',2); grid on; hold on
plot(diff(a.steeringVelocityBuffer(2,:)),'Color','b','LineWidth',2)
plot(diff(a.steeringVelocityBuffer(3,:)),'Color',[0,114,0]./256,'LineWidth',2)
% plot(ideal.steeringVelocity(1,:),'Color','r','LineWidth',1)
% plot(ideal.steeringVelocity(2,:),'Color','b','LineWidth',1)
% plot(ideal.steeringVelocity(3,:),'Color',[0,114,0]./256,'LineWidth',1)
title('angularAcc')

subplot(428)
plot(g.omegaMaxErrorIdeal); grid on; hold on
plot(omegaMaxErrorBuffer)
% plot(minPercentageBuffer)
title('maxOmegaError')% + minPercentage')
legend('maxOmegaErrorIdeal','maxOmegaError')%,'minPercentage')

%% 
figure
subplot(211)
plot(a.betaSetpointBuffer'); grid on; hold on
% plot(rampedAnglesBuffer')
% plot(betaModBuffer')
legend('setpoints','ramped','mod')

subplot(212)
plot(minPercentageBuffer); grid on 
title('minPercentage')

%%
% figure
% subplot(131)
% plot(omegaMaxMod)
% title('maxErrorMod')
% subplot(132)
% plot(vModBuffer'); grid on; hold on
% plot(g.vRampedBuffer')
% plot(vScaledBuffer')
% title('vMod')
% subplot(133)
% plot(betaModBuffer')
% title('betaMod')