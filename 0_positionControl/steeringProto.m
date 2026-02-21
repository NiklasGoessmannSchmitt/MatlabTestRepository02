close all
clear all
clc

a.pivotValues(1).angularPosition = 0; 
a.pivotValues(2).angularPosition = 0; 
a.pivotValues(3).angularPosition = 0;

a.betaSetpoints(1) = pi/2;
a.betaSetpoints(2) = pi/3; 
a.betaSetpoints(3) = pi/4; 

for i = 1:10000

    steeringError(1,i) = a.betaSetpoints(1) - a.pivotValues(1).angularPosition;
    steeringError(2,i) = a.betaSetpoints(2) - a.pivotValues(2).angularPosition;
    steeringError(3,i) = a.betaSetpoints(3) - a.pivotValues(3).angularPosition;
    maxSteeringError(i) = max(abs(steeringError(:,i))); 
    partError(:,i) = steeringError(:,i)./maxSteeringError(i); 
    
    setpointSteeringVelocity(1) = steeringError(1,i) * 1.5; 
    setpointSteeringVelocity(2) = steeringError(2,i) * 1.5;
    setpointSteeringVelocity(3) = steeringError(3,i) * 1.5;

%     setpointSteeringVelocity(1) = abs(partError(1,i)) * steeringError(1,i) * 1; 
%     setpointSteeringVelocity(2) = abs(partError(2,i)) * steeringError(2,i) * 1;
%     setpointSteeringVelocity(3) = abs(partError(3,i)) * steeringError(3,i) * 1;
    
    a.pivotValues(1).angularPosition = a.pivotValues(1).angularPosition + setpointSteeringVelocity(1)*1e-3; 
    a.pivotValues(2).angularPosition = a.pivotValues(2).angularPosition + setpointSteeringVelocity(2)*1e-3; 
    a.pivotValues(3).angularPosition = a.pivotValues(3).angularPosition + setpointSteeringVelocity(3)*1e-3; 

end

figure 
plot(steeringError'); grid on

% Ziel: Fehler konvergieren gleichzeitig und -förmig nach NULL 