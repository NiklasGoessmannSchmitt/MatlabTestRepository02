function [vl, vr] = LSimoveC_UC_MoveCtrl_Tricycle_Passive_W2M(velocityVs, velocitySteering, actAngle, VaVehicle, pivotParam)


        tempVelocitySetpoint.x = velocityVs;
        tempVelocitySetpoint.y = 0;
        tempVelocitySetpoint.a = velocitySteering;
    
        if pivotParam.ringmount > 0.0 %%&& pivotParam.steeringGearRatio == 1.0 
            tempVaMmS = tempVelocitySetpoint.a * pivotParam.ringmount;
        elseif pivotParam.ringmount == 0 %%&& pivotParam.steeringGearRatio > 0.0 
            tempVaMmS = tempVelocitySetpoint.a * pivotParam.steeringGearRatio * (pivotParam.drivingWheelDiameter / 2.0) / pivotParam.drivingGearRatio; 
        else
            % error message CP
        end
    
        tempVlMmS = tempVelocitySetpoint.x - tempVaMmS; 
        tempVrMmS = tempVelocitySetpoint.x + tempVaMmS; 
        
        tempDistXWheel = cos(3.141592 / 2 + actAngle) * pivotParam.ringmount;
        tempDistYWheel = sin(3.141592 / 2 + actAngle) * pivotParam.ringmount;
        tempVxWheel =  - VaVehicle * tempDistYWheel;
        tempVyWheel =  + VaVehicle * tempDistXWheel;
        tempDeltaVWheel = sqrt(tempVxWheel*tempVxWheel + tempVyWheel*tempVyWheel);
        if VaVehicle >= 0 
            tempDeltaVWheel = - tempDeltaVWheel;
        else
            tempDeltaVWheel = + tempDeltaVWheel;
        end
        if tempVlMmS == 0.0 && tempVrMmS == 0.0
            tempDeltaVWheel = 0.0;
        end
        vl = (tempVlMmS + tempDeltaVWheel) / 2.0 / pi / (pivotParam.diameter / 2) * 60.0 * pivotParam.gearratio;
        vr = (tempVrMmS - tempDeltaVWheel) / 2.0 / pi / (pivotParam.diameter / 2) * 60.0 * pivotParam.gearratio;
       
    
%     if (tempVelocitySetpointLeft = 0) && (tempVelocitySetpointRight = 0) && (tempVelocitySetpoint.x = 0) 
%         cmdStandstill = true;
%     else
%         cmdStandstill = false;
%     end
%     
%     vl = tempVelocitySetpointLeft;
%     vr = tempVelocitySetpointRight;

end