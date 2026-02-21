function general = initGeneralValues(xAGV,yAGV,aAGV,simSteps)

general.curMapPos = [xAGV, yAGV, aAGV];
general.oldMapPos = general.curMapPos;
general.simSteps = simSteps;

general.vRampedBuffer = zeros(3, simSteps);
general.vSetBuffer = zeros(3, simSteps); 
general.vCurrentBuffer = zeros(3,simSteps); 

general.trackDeviationBuffer = zeros(3, general.simSteps);