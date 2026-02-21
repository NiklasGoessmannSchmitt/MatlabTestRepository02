function vtshandles = plotVTS(carrotGlobal, agvPose, target) 

% Define Target
carrotFrame = [carrotGlobal(1)+0.1, carrotGlobal(2);       ... % xAxis
               carrotGlobal(1)    , carrotGlobal(2)+0.1]'; ... % yAxis

% Rotate around Target root 
center = repmat([carrotGlobal(1); carrotGlobal(2)], 1, length(carrotFrame));
R = [cos(carrotGlobal(3)) -sin(carrotGlobal(3)); sin(carrotGlobal(3)) cos(carrotGlobal(3))];
targetFrameRot = (R*(carrotFrame - center) + center)';

vtshandles.target = plot(carrotGlobal(1), carrotGlobal(2), 'rx', 'LineWidth', 3); % ANS-target
vtshandles.targetCoSystem = plot([carrotGlobal(1) targetFrameRot(1,1)], [carrotGlobal(2) targetFrameRot(1,2)],'k',...'LineWidth',1.5,...
        [carrotGlobal(1) targetFrameRot(2,1)], [carrotGlobal(2) targetFrameRot(2,2)],'k','LineWidth',1.5); % Target coordinate system

% vtshandles.asdf = plot([agvPose(1),agvPose(2)])