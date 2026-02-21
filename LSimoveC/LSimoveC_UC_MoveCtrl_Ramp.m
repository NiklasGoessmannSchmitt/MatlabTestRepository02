function [vSetpoint_out, state, rampenfaktor, accBit, decBit, ReleaseDrives] = LSimoveC_UC_MoveCtrl_Ramp (stopReq, accRel, veloRel, vSetpoint_in, vMax, aAccMax, aDecMax, samplingTime)
% *******************************************************************
% [vSetpoint_out, Rueckwaerts, state, Rampenfaktor, accBit, decBit, ReleaseDrives] 
%               = LSimoveC_UC_MoveCtrl_Ramp (StopReq, AccRel, VeloRel, vSetpoint_in, vMax, aAccMax, aDecMax, SamplingTime)
%  Description
%   This function block implements the XYA ramping respecting: 
%       - maximum velocities
%       - maximum accelerations
%       - maximum decelerations
% 
%   Furthermore, the function block two more features: 
%       1. VELOCITIES other than ZERO
%          are only allowed when 'VelocityReleased' == true
%       2. ACCELERATIONS other than ZERO
%          are only allowed when 'AccelerationReleased' == true
% 
%  Version
%  V1.0 2019-06-17 CP: Initial Draft
%  V3.0 2020-02-17 CP: Acceleration adjustments for pivot settings
%  V3.0 2020-03-18 DG: Remove of BeschleunigungResultierend, Correct usage
%                      of StopRequest
%  Autors
%  CP: Christoph Prummer
%  DG: Daniel Gauglitz
% 
%  © Alle Rechte vorbehalten/All rights reserved, Siemens AG, 2018
% *******************************************************************)

%% Persistent Variables (static)
persistent statSollwertRampeX 
persistent statSollwertRampeY
persistent statSollwertRampeA
persistent statSollwertRampeXY
persistent statSollwertRampeAltX
persistent statSollwertRampeAltY
persistent statSollwertRampeAltA
persistent statSollwertRampeAltXY
persistent statRueckwaerts

if isempty(statRueckwaerts)
    statRueckwaerts = false;
end
if isempty(statSollwertRampeAltX)
    statSollwertRampeAltX = 0;
end
if isempty(statSollwertRampeAltY)
    statSollwertRampeAltY = 0;
end
if isempty(statSollwertRampeAltA)
    statSollwertRampeAltA = 0;
end
if isempty(statSollwertRampeX)
    statSollwertRampeX = 0;
end
if isempty(statSollwertRampeY)
    statSollwertRampeY = 0;
end
if isempty(statSollwertRampeA)
    statSollwertRampeA = 0;
end
if isempty(statSollwertRampeXY)
    statSollwertRampeXY = 0;
end
if isempty(statSollwertRampeAltXY)
    statSollwertRampeAltXY = 0;
end

%% VeloRel
if veloRel && ~stopReq 
    tempSetpoint.x = vSetpoint_in(1);
    tempSetpoint.y = vSetpoint_in(2);
    tempSetpoint.a = vSetpoint_in(3); 
else
    tempSetpoint.x = 0;
    tempSetpoint.y = 0;
    tempSetpoint.a = 0; 
end

%%
state = 0;
%% ==========================================================================================================================
% CHECK 1: Setpoint is not above velocity limits
%==========================================================================================================================
% Velocities in relation to the maximum values
if vMax.x ~= 0 
    tempRatioVelo.x = abs(tempSetpoint.x / vMax.x); %// assuming max is valid for + and - direction
else
    tempRatioVelo.x = 0;
    tempSetpoint.x = 0;
end
if vMax.y ~= 0 
    tempRatioVelo.y = abs(tempSetpoint.y / vMax.y); %// assuming max is valid for + and - direction
else
    tempRatioVelo.y = 0;
    tempSetpoint.y = 0;
end
if vMax.a ~= 0 
    tempRatioVelo.a = abs(tempSetpoint.a / vMax.a); %// assuming max is valid for + and - direction
else
    tempRatioVelo.a = 0;
    tempSetpoint.a = 0;
end
tempvMaxXY = vMax.xy; 
if tempvMaxXY < vMax.x || tempvMaxXY < vMax.y 
    tempvMaxXY = max([vMax.x, vMax.y]); 
end
if tempvMaxXY ~= 0 
    tempRatioVelo.xy = abs(sqrt(tempSetpoint.x^2 +tempSetpoint.y^2) / tempvMaxXY); 
else
    tempRatioVelo.xy = 0; 
end

if max( [tempRatioVelo.x, tempRatioVelo.y, tempRatioVelo.a] ) > 1 % at least one velocity maximum is exceeded
    
    if tempRatioVelo.x > tempRatioVelo.y && tempRatioVelo.x > tempRatioVelo.a && tempRatioVelo.x > tempRatioVelo.xy % x is limiting 
        
        tempSetpoint.x = tempSetpoint.x / abs(tempSetpoint.x) * vMax.x;
        tempSetpoint.y = vSetpoint_in(2) * tempSetpoint.x / vSetpoint_in(1);
        tempSetpoint.a = vSetpoint_in(3) * tempSetpoint.x / vSetpoint_in(1);
        state = 14;
        
    elseif tempRatioVelo.y > tempRatioVelo.a && tempRatioVelo.y > tempRatioVelo.xy % y is limiting
        
        tempSetpoint.y = tempSetpoint.y / abs(tempSetpoint.y) * vMax.y;
        tempSetpoint.x = vSetpoint_in(1) * tempSetpoint.y / vSetpoint_in(2);
        tempSetpoint.a = vSetpoint_in(3) * tempSetpoint.y / vSetpoint_in(2);
        state = 15;
        
    elseif tempRatioVelo.a > tempRatioVelo.xy % a is limiting
        
        tempSetpoint.a = tempSetpoint.a / abs(tempSetpoint.a) * vMax.a;
        tempSetpoint.x = vSetpoint_in(1) * tempSetpoint.a / vSetpoint_in(3);
        tempSetpoint.y = vSetpoint_in(2) * tempSetpoint.a / vSetpoint_in(3);
        state = 16; 
        
    else % xy is limiting 
        
        tempSetpoint.x = tempSetpoint.x / tempRatioVelo.xy;
        tempSetpoint.y = vSetpoint_in(2) / tempRatioVelo.xy;
        tempSetpoint.a = vSetpoint_in(3) / tempRatioVelo.xy;
        state = 17; 
        
    end
    
end % at least one velocity maximum is exceeded

%% Einzelne Rampenschritte ausrechnen
[tempSetpointRamped.x, accBit.x, decBit.x] = fbRampe(tempSetpoint.x, statSollwertRampeX, aAccMax.x, aDecMax.x, samplingTime);
[tempSetpointRamped.y, accBit.y, decBit.y] = fbRampe(tempSetpoint.y, statSollwertRampeY, aAccMax.y, aDecMax.y, samplingTime);
[tempSetpointRamped.a, accBit.a, decBit.a] = fbRampe(tempSetpoint.a, statSollwertRampeA, aAccMax.a, aDecMax.a, samplingTime);

tempaAccMax.xy = aAccMax.xy;
if tempaAccMax.xy < aAccMax.x || tempaAccMax.xy < aAccMax.y 
    tempaAccMax.xy = max([aAccMax.x, aAccMax.y]);
end
tempaDecMax.xy = aDecMax.xy;
if tempaDecMax.xy < aDecMax.x || tempaDecMax.xy < aDecMax.y
    tempaDecMax.xy = max([aDecMax.x, aDecMax.y]);
end
[tempSetpointRamped.xy, accBit.xy, decBit.xy] = fbRampe(sqrt(tempSetpoint.x^2 + tempSetpoint.y^2), statSollwertRampeXY, tempaAccMax.xy, tempaDecMax.xy, samplingTime); 

% Regelabweichungen und Schrittweiten ausrechnen
tempSollwertabweichung.x = tempSetpoint.x - statSollwertRampeAltX;
tempSollwertabweichung.y = tempSetpoint.y - statSollwertRampeAltY;
tempSollwertabweichung.a = tempSetpoint.a - statSollwertRampeAltA;
tempSollwertabweichung.xy = sqrt(tempSetpoint.x^2 + tempSetpoint.y^2) - statSollwertRampeAltXY; 

tempStep.x = tempSetpointRamped.x - statSollwertRampeAltX;
tempStep.y = tempSetpointRamped.y - statSollwertRampeAltY;
tempStep.a = tempSetpointRamped.a - statSollwertRampeAltA;
tempStep.xy = tempSetpointRamped.xy - statSollwertRampeAltXY; 

% Anteilige Erhöhung von altem Wert auf Sollwert bestimmen
if tempSollwertabweichung.x ~= 0 
    tempAnteil.x = tempStep.x / tempSollwertabweichung.x;
else
    tempAnteil.x = 1;
end

if tempSollwertabweichung.y ~= 0
    tempAnteil.y = tempStep.y / tempSollwertabweichung.y;
else
    tempAnteil.y = 1;
end

if tempSollwertabweichung.a ~= 0 
    tempAnteil.a = tempStep.a / tempSollwertabweichung.a;
else
    tempAnteil.a = 1;
end

if tempSollwertabweichung.xy ~= 0 
    tempAnteil.xy = tempStep.xy / tempSollwertabweichung.xy; 
else
    tempAnteil.xy = 1; 
end

% Minimale anteilige Schrittweite herausfinden
tempRampenfaktor = min( [1, tempAnteil.x, tempAnteil.y, tempAnteil.a, tempAnteil.xy] );

if (tempRampenfaktor == tempAnteil.xy) && ...
        abs(tempAnteil.xy - tempAnteil.x) > 1e-5 && ...
        tempRampenfaktor < 1
    ;
end 

% Alle Sollwerte entsprechend minimaler anteiliger Schrittweite beaufschlagen, um gleichmäßiges Anfahren zu ermöglichen
tempWunschSollwert.x = statSollwertRampeAltX + tempRampenfaktor * tempSollwertabweichung.x;
tempWunschSollwert.y = statSollwertRampeAltY + tempRampenfaktor * tempSollwertabweichung.y;
tempWunschSollwert.a = statSollwertRampeAltA + tempRampenfaktor * tempSollwertabweichung.a;
tempWunschSollwert.xy = statSollwertRampeAltXY + tempRampenfaktor * tempSollwertabweichung.xy; 

% Nur bei entsprechender Rampenfreigabe darf die Geschwindigkeit betragsmäßig erhöht werden
if accRel || abs(tempWunschSollwert.x) <= abs(statSollwertRampeAltX) 
    statSollwertRampeX = tempWunschSollwert.x;
else
    statSollwertRampeX = statSollwertRampeAltX;
end

if accRel || abs(tempWunschSollwert.y) <= abs(statSollwertRampeAltY)
    statSollwertRampeY = tempWunschSollwert.y;
else
    statSollwertRampeY = statSollwertRampeAltY;
end

if accRel || abs(tempWunschSollwert.a) <= abs(statSollwertRampeAltA) 
    statSollwertRampeA = tempWunschSollwert.a;
else
    statSollwertRampeA = statSollwertRampeAltA;
end

if accRel || abs(tempWunschSollwert.xy) <= abs(statSollwertRampeAltXY) 
    statSollwertRampeXY = tempWunschSollwert.xy; 
else
    statSollwertRampeXY = statSollwertRampeAltXY; 
end

statSollwertRampeAltX = statSollwertRampeX;
statSollwertRampeAltY = statSollwertRampeY;
statSollwertRampeAltA = statSollwertRampeA;
statSollwertRampeAltXY = statSollwertRampeXY;

% Ausgänge schreiben
vSetpoint_out(1) = statSollwertRampeX;
vSetpoint_out(2) = statSollwertRampeY;
vSetpoint_out(3) = statSollwertRampeA;

rampenfaktor = tempRampenfaktor;

if vSetpoint_out(1) ~= 0 || vSetpoint_out(2) ~= 0 || vSetpoint_out(3) ~= 0 
    ReleaseDrives = true;
else
    ReleaseDrives = false; 
end
end