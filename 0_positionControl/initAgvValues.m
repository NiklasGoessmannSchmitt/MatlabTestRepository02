function agv = initAgvValues(general, n, agvType, motor, agvMeasures, sensor, pivotParam, pivotValues)

agv.type = agvType;

if strcmp(agvType,'Tricycle') || strcmp(agvType,'TricycleRear') || ...
        strcmp(agvType, 'PivotActive') || strcmp(agvType, 'PivotPassive') || ...
        strcmp(agvType, 'SteeredDiff_SteerMaster') || strcmp(agvType, 'SteeredDiff_DiffMaster') || ... 
        strcmp(agvType, 'SteeredDiff_OneWheel')
        
    
    agv.vsCurrentBuffer = zeros(n, general.simSteps);
    agv.vsSetpointBuffer = zeros(n, general.simSteps); 
    
    agv.steeringVelocitySetpointBuffer = zeros(n, general.simSteps); 
    agv.steeringVelocityBuffer = zeros(n, general.simSteps); 
    
    agv.betaSetpoints = zeros(n, 1);
    agv.betaCurrentBuffer = zeros(n, general.simSteps);
    agv.betaSetpointBuffer = zeros(n, general.simSteps);
    agv.oldBeta = zeros(n, 1); 
    
    agv.steeringBuffer = zeros(n, general.simSteps); 
    agv.movementBuffer = zeros(n, general.simSteps); 
    
    agv.z = zeros(n,length(motor(2,:))-1);
    
    agv.intePartoldY = 0; 
    agv.intePartoldA = 0; 
    agv.betaSetpoints = 0;
    
    agv.setPointLeft = zeros(n, general.simSteps);
    agv.setPointRight = zeros(n, general.simSteps);
    
elseif strcmp(agvType,'Diff')
    
    agv.intePartoldY = 0; 
    agv.intePartoldA = 0;  
    agv.betaSetpoints = 0;
    agv.zl = zeros(1,length(motor(2,:))-1);
    agv.zr = zeros(1,length(motor(2,:))-1);
    agv.vlBuffer = zeros(1, general.simSteps);
    agv.vrBuffer = zeros(1, general.simSteps);
    
elseif strcmp(agvType,'Mecanum')
    
    agv.intePartoldY = 0; 
    agv.intePartoldA = 0; 
    agv.betaSetpoints = 0;
    agv.zl = zeros(1,length(motor(2,:))-1);
    agv.zr = zeros(1,length(motor(2,:))-1);
    agv.vlBuffer = zeros(1, general.simSteps);
    agv.vrBuffer = zeros(1, general.simSteps); 
    
end

if strcmp(agvType, 'PivotPassive') 
    agv.vlBuffer = zeros(n, general.simSteps);
    agv.vrBuffer = zeros(n, general.simSteps);
end

agv.n = n;

agv.agvMeasures = agvMeasures; 
agv.statSteering = false; 
agv.statMovement = false; 
agv.rampingFactorSteering = 1; 

agv.sensor = sensor; 
agv.pivotParam = pivotParam;
agv.pivotValues = pivotValues;