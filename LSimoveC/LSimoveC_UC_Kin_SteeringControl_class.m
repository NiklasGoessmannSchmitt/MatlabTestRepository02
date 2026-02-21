classdef LSimoveC_UC_Kin_SteeringControl_class < handle
    properties
        statSetpointSteeringSpeed {mustBeNumeric}
        omegaMaxRadS {mustBeNumeric}
        steerShortestWay {mustBeNumericOrLogical}
        kp {mustBeNumeric}
        samplingTime {mustBeNumeric}
        maxAccWheel {mustBeNumeric}
    end
    methods
        %Constructor     
        function obj = LSimoveC_UC_Kin_SteeringControl_class(iOmegaMaxRadS, iMaxAccelerationWheel, steerShortestWay, iKp, samplingTime)
            obj.statSetpointSteeringSpeed = 0; 
            obj.omegaMaxRadS = iOmegaMaxRadS;
            obj.maxAccWheel = iMaxAccelerationWheel;
            obj.steerShortestWay = steerShortestWay;
            obj.kp = iKp;
            obj.samplingTime = samplingTime;
        end

        %Methods
        function SetpointSteeringVelocity = SteeringControl(obj,Release,AngleSetpoint, AngleCurrent)
            
            tempControlDeviation = AngleSetpoint - AngleCurrent;
            % Determine shortest way for steering
            if tempControlDeviation >= 0
                if tempControlDeviation > pi && obj.steerShortestWay 
                    tempSteeringDirection = -1;
                    tempControlDeviation = 2 * pi - tempControlDeviation;
                else
                    tempSteeringDirection = 1;
                end
                % Control deviation negative
            else
                if tempControlDeviation < -pi && obj.teerShortestWay 
                    tempSteeringDirection = 1;
                    tempControlDeviation = 2 * pi + tempControlDeviation;
                else
                    tempSteeringDirection = -1;
                    tempControlDeviation = - tempControlDeviation;
                end
            end

            % Check if control deviation exceeds tolerance
            tempSteeringSpeed = 0;
            if Release 
                % Proportional part
                tempProportional = obj.kp * tempSteeringDirection * tempControlDeviation;
                tempSteeringSpeed = tempProportional / obj.samplingTime;
            end

            % Truncate to maximum speed
            if tempSteeringSpeed > 0 && tempSteeringSpeed > obj.omegaMaxRadS 
                tempSteeringSpeed = obj.omegaMaxRadS;
            elseif tempSteeringSpeed < 0 && abs(tempSteeringSpeed) > obj.omegaMaxRadS 
                tempSteeringSpeed = - obj.omegaMaxRadS;
            end

            obj.statSetpointSteeringSpeed = fbRampe(tempSteeringSpeed, ...
                                                obj.statSetpointSteeringSpeed, ...
                                                obj.maxAccWheel, ...
                                                obj.maxAccWheel, ...
                                                obj.samplingTime);

            SetpointSteeringVelocity = obj.statSetpointSteeringSpeed;
        end
    end
end