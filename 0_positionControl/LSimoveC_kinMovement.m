function [v, internalPosition, done] = LSimoveC_kinMovement(targetPosAgvFrame, agvPose)

STATE_INIT = 0; 
STATE_DRIVE = 1; 
STATE_FINISH = 2; 

done = false; 

persistent statInitialAgvPosition 
persistent statAGVPosition
persistent statTarget
persistent statState

if isempty(statInitialAgvPosition)
    statInitialAgvPosition = [0,0,0];
    statAGVPosition = [0,0,0];
    statTarget = [0,0,0]; 
    statState = 0; 
end
v = [0,0,0]; 

switch statState 
    case STATE_INIT

        statTarget(1) = targetPosAgvFrame(1);
        statTarget(2) = targetPosAgvFrame(2);
        statTarget(3) = wrapToPi(targetPosAgvFrame(3));
        statInitialAgvPosition = agvPose;

        statState = STATE_DRIVE;
  
    case STATE_DRIVE
        xError = statTarget(1) - statAGVPosition(1);
        yError = statTarget(2) - statAGVPosition(2);
        aError = statTarget(3) - statAGVPosition(3);
        v(1) = 1.0* (xError*cos(-statAGVPosition(3)) - yError*sin(-statAGVPosition(3)));
        v(2) = 1.0* (xError*sin(-statAGVPosition(3)) + yError*cos(-statAGVPosition(3)));

        if aError > 0 
            v(3) = 1.0* sqrt(2*0.5*abs(aError));
        else
            v(3) = -1.0* sqrt(2*0.5*abs(aError));
        end

        if abs(xError) < 10e-3 ...
            && abs(yError) < 10e-3 ...
            && abs(aError) < 10e-3
        
           statState = STATE_FINISH;
        end                 
        
    case STATE_FINISH
            done = true;        
end

statAGVPosition(1) = (agvPose(1) - statInitialAgvPosition(1)) * cos(-statInitialAgvPosition(3)) - (agvPose(2) - statInitialAgvPosition(2)) * sin(-statInitialAgvPosition(3));
statAGVPosition(2) = (agvPose(1) - statInitialAgvPosition(1)) * sin(-statInitialAgvPosition(3)) + (agvPose(2) - statInitialAgvPosition(2)) * cos(-statInitialAgvPosition(3));
statAGVPosition(3) = wrapToPi(agvPose(3) - statInitialAgvPosition(3));

internalPosition = statAGVPosition; 