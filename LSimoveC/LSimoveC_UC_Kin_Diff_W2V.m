function [v, pos] = LSimoveC_UC_Kin_Diff_W2V(encLeft, encRight, encMax, encMin, startup, t)

persistent statEncLeftBefore
persistent statEncRightBefore
persistent statPositionPrev

if isempty(statEncLeftBefore) 
    statEncLeftBefore = 0; 
    statEncRightBefore = 0; 
    statPositionPrev = zeros(1,3); 
end

	tempEncoderImpulseProUmdrehung = 500000; %PivotParam.DriveParam.NumberOfIncrementsMotor);
% 	tempCurrentVelocityLeft := #VelocityLeft;
% 	tempCurrentVelocityRight := #VelocityRight;
	tempSystemRuntimeS = t; 
	
	%% REGION calculate some needed values and factors
	    tempDrehkranzradius = 320;% PivotParam.RadiusRingmount);
	    
        wheelDiameter = 100; 
        gearRatio = 33; 
	    tempEncoderIncToMmFactor = 2.0 * pi * wheelDiameter / 2 / tempEncoderImpulseProUmdrehung / gearRatio;
	    
% 	    #tempRpmToMmS := REAL#2.0
% 	    * #PI
% 	    * #PivotParam.DriveParam.WheelDiameter/2
% 	    / REAL#60.0
% 	    / #PivotParam.DriveParam.GearRatio;
	    
	%% _calculate velocity from drive values_
	    
% 	    IF #CalcMode = 0 THEN // CalcMode "0": wheel velocities [rpm]
% 	        
% 	        #Velocity.x := ((#tempCurrentVelocityRight + #tempCurrentVelocityLeft) / 2.0) * #tempRpmToMmS;
% 	        #Velocity.y := 0.0;
% 	        #Velocity.a := (#tempCurrentVelocityRight - #tempCurrentVelocityLeft) * #tempRpmToMmS / (2.0 * #tempDrehkranzradius);
	        
% 	    ELSIF #CalcMode = 1 THEN // CalcMode "1": encoder values [-]
	        
            if startup 
                statEncLeftBefore = encLeft; 
                statEncRightBefore = encRight; 
            end

% 	        // get left encoder ticks
            tempEncTicksLeft = LSimoveC_OverflowDetection(encLeft, statEncLeftBefore,encMax, encMin); 
            tempDistanceLeftFromEncoder = tempEncTicksLeft * tempEncoderIncToMmFactor; 
	        tempVelocityLeftFromEncoder = tempDistanceLeftFromEncoder / tempSystemRuntimeS;
	        
% 	        // get right encoder ticks
            tempEncTicksRight = LSimoveC_OverflowDetection(encRight, statEncRightBefore,encMax, encMin); 
            tempDistanceRightFromEncoder = tempEncTicksRight * tempEncoderIncToMmFactor; 
	        tempVelocityRightFromEncoder = tempDistanceRightFromEncoder / tempSystemRuntimeS;

	        v(1) = (tempVelocityRightFromEncoder + tempVelocityLeftFromEncoder) / 2.0;
	        v(2) = 0.0;
	        v(3) = (tempVelocityRightFromEncoder - tempVelocityLeftFromEncoder) / (2.0 * tempDrehkranzradius);
            
	        statEncLeftBefore = encLeft;
	        statEncRightBefore = encRight;
            
% 	    ELSIF #CalcMode = 2 THEN // CalcMode "2": wheel position [mm]
% 	        
% 	        #tempDistanceRightFromEncoder := UDINT_TO_REAL(#DistanceRight);
% 	        #tempDistanceLeftFromEncoder := UDINT_TO_REAL(#DistanceLeft);
% 	        #tempVelocityRightFromEncoder := #tempDistanceRightFromEncoder / #tempSystemRuntimeS;
% 	        #tempVelocityLeftFromEncoder := #tempDistanceLeftFromEncoder / #tempSystemRuntimeS;
% 	        
% 	        #Velocity.x := (#tempVelocityRightFromEncoder + #tempVelocityLeftFromEncoder) / 2.0;
% 	        #Velocity.y := 0.0;
% 	        #Velocity.a := (#tempVelocityRightFromEncoder - #tempVelocityLeftFromEncoder) / (2.0 * #tempDrehkranzradius);
% 	        
% 	    ELSE
% 	        #Velocity.x := 0.0;
% 	        #Velocity.y := 0.0;
% 	        #Velocity.a := 0.0;
% 	        
% 	    END_IF;
% 	    
% 	    if ~enable 
% 	        v(1) = 0.0;
% 	        v(2) = 0.0;
% 	        v(3) = 0.0;
%         end
%  	    
% 	    #Velocity_x := #Velocity.x;
% 	    #Velocity_y := #Velocity.y;
% 	    #Velocity_a := #Velocity.a;
% 	    #Velocity_aDeg := #Velocity.a * 180 / #PI;
% 	END_REGION
% 	//--------------------------------------------------------------------------------------------------------------
% 	REGION _calculate position from drive values_
% 	    IF #ResetPosition THEN
% 	        #Position.a := 0;
% 	        #Position.x := 0;
% 	        #Position.y := 0;
% 	        #statIstPositionVorher := #Position;
% 	    END_IF;
% 	    
% 	    IF #SetAbsoluteDistance THEN
% 	        #statTravelledDistanceBefore := #AbsoluteDistanceValue;
% 	    END_IF;
% 	    IF #SetRefDistance THEN
% 	        #statReferencedDistanceBefore := #RefDistanceValue;
% 	    END_IF;
% 	    
% 	    IF #CalcMode = 0 THEN // using RPM
% 	        #Position.a := "MOD_REAL"(IN := #statIstPositionVorher.a + #Velocity.a * #tempSystemRuntimeS, "DIV" := #BOGENMASS_MAX);
% 	        #Position.x := #statIstPositionVorher.x + (#Velocity.x * #tempSystemRuntimeS) * COS(#Position.a);
% 	        #Position.y := #statIstPositionVorher.y + (#Velocity.x * #tempSystemRuntimeS) * SIN(#Position.a); //Verwendung von #IstGeschwindigkeit.x ist richtig!
% 	        #AbsoluteDistance := #statTravelledDistanceBefore + (#Velocity.x * #tempSystemRuntimeS);
% 	        #ReferencedDistance := #statReferencedDistanceBefore + REAL_TO_LREAL(#Velocity.x * #tempSystemRuntimeS);
% 	        
% 	    ELSIF #CalcMode = 1 OR #CalcMode = 2 THEN // using encoder or distance
            % Wertebereich pos(3) = [0..2pi]
	        pos(3) = mod(statPositionPrev(3) + (tempDistanceRightFromEncoder - tempDistanceLeftFromEncoder) / (2.0 * tempDrehkranzradius), 2*pi);
	        pos(1) = statPositionPrev(1) + (tempDistanceRightFromEncoder + tempDistanceLeftFromEncoder) / 2.0 * cos(pos(3));
	        pos(2) = statPositionPrev(2) + (tempDistanceRightFromEncoder + tempDistanceLeftFromEncoder) / 2.0 * sin(pos(3)); 
% 	        #AbsoluteDistance := #statTravelledDistanceBefore + (#tempDistanceRightFromEncoder + #tempDistanceLeftFromEncoder) / 2.0;
% 	        #ReferencedDistance := #statReferencedDistanceBefore + (#tempDistanceRightFromEncoder + #tempDistanceLeftFromEncoder) / 2.0;
% 	        
% 	    ELSE // invalid choice
% 	        ;
% 	    END_IF;
% 	        
	    statPositionPrev = pos;
% 	    #statTravelledDistanceBefore := #AbsoluteDistance;
% 	    #statReferencedDistanceBefore := #ReferencedDistance;
% 	    
% 	    #Position_x := #Position.x;
% 	    #Position_y := #Position.y;
% 	    #Position_a := #Position.a;
% 	    #Position_aDeg := #Position.a * 180 / #PI;
% 	END_REGION
% 	
% 	// REGION FIR-Filter
% 	//     // FIR - filter of 50th order [0..49] || moving average filter
% 	//     FOR #tempIndex := 1 TO 99 DO
% 	//         #statVelocityXCoeffs[#tempIndex - 1] := #statVelocityXCoeffs[#tempIndex];
% 	//         #statVelocityACoeffs[#tempIndex - 1] := #statVelocityACoeffs[#tempIndex];
% 	//     END_FOR;
% 	//     #statVelocityXCoeffs[99] := #Velocity_RAW.x;
% 	//     #statVelocityACoeffs[99] := #Velocity_RAW.a;
% 	    
% 	//     #statVelocityXCoeffsSUM := 0;
% 	//     #statVelocityACoeffsSUM := 0;
% 	    
% 	//     FOR #tempIndex := 0 TO 99 DO
% 	//         #statVelocityXCoeffsSUM := #statVelocityXCoeffsSUM + #statVelocityXCoeffs[#tempIndex];
% 	//         #statVelocityACoeffsSUM := #statVelocityACoeffsSUM + #statVelocityACoeffs[#tempIndex];
% 	//     END_FOR;
% 	//     #Velocity.x := #statVelocityXCoeffsSUM / 100;
% 	//     #Velocity.a := #statVelocityACoeffsSUM / 100;
% 	    
% 	//     #Velocity_x := #Velocity.x;
% 	//     #Velocity_y := #Velocity.y;
% 	//     #Velocity_a := #Velocity.a;
% 	//     #Velocity_aDeg := #Velocity.a * 180 / #PI;
% 	    
% 	// END_REGION