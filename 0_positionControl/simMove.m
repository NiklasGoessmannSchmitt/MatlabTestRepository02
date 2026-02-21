close all
clear all
clc

%%

agvMeasures.front = 1; 
agvMeasures.back = 0.5; 
agvMeasures.left = 0.5; 
agvMeasures.right = 0.5; 
j = 0;
samplingTime = 1e-3;
vCurrent = [0,0,0];
simSteps = 10000; 

% targetPosAgvFrame = [1,0,0];
% targetPosAgvFrame = [1,1,1];
% targetPosAgvFrame = [1,0,pi/2];
% targetPosAgvFrame = [1,1,pi/2];
targetPosAgvFrame = [0,0,3*pi/2];
% targetPosAgvFrame = [1,2,pi];
% targetPosAgvFrame = [1,2,-pi];

% curMapPos = [0,0,0.001];
% curMapPos = [0,0,-0.001];
curMapPos = [1,0,pi+0.2];
oldMapPos = curMapPos;

mapPosBuffer = zeros(3,simSteps); 

%%
%% RUN simulation 
for i = 1:simSteps

    [vCurrent, internalPosition, done] = LSimoveC_kinMovement(targetPosAgvFrame, curMapPos);

    if done 
        disp('done')
        break; 
    end

    curMapPos(3) = oldMapPos(3) + vCurrent(3) * samplingTime;
    curMapPos(3) = wrapToPi(curMapPos(3));

    curMapPos(1) = oldMapPos(1) ...
                   + (vCurrent(1) * samplingTime) * cos(curMapPos(3) ) ...
                   - (vCurrent(2) * samplingTime) * sin(curMapPos(3) ); 
    curMapPos(2) = oldMapPos(2) ...
                   + (vCurrent(1) * samplingTime) * sin(curMapPos(3) ) ...
                   + (vCurrent(2) * samplingTime) * cos(curMapPos(3) ); 

    if mod(i,10) == 0
            
        if j == 0 
            fGlobalMap = figure('units','normalized','outerposition',[0 0 1 1]);
        end
        j = j + 1;
        pause(0.001)
        figure(fGlobalMap)  
        
        if exist('handles','var')
            if isfield(handles,'agvBody');delete(handles.agvBody);end
            if isfield(handles,'agvCenter');delete(handles.agvCenter);end
            if isfield(handles,'agvCoSystem');delete(handles.agvCoSystem);end
            if isfield(handles,'pivotCenter');delete(handles.pivotCenter);end
            if isfield(handles,'pivotWheel');delete(handles.pivotWheel);end
        end

        if exist('I','var') % does an image file exist?
            if ~isfield(hh, 'image') % has an image been printed?
                hh.image = image([0 30],[0 26], I); hold on;
                uistack(hh.image,'bottom')
            end    
        end  
        daspect([1 1 1])
    
        % Define AGV parts
        %              X                                  Y
        agvFrame = [curMapPos(1)+0.1              , curMapPos(2);                                  ... % xAxis
                    curMapPos(1)                  , curMapPos(2)+0.1;                              ... % yAxis
                    curMapPos(1)+agvMeasures.front, curMapPos(2)-agvMeasures.left;                 ... % front left
                    curMapPos(1)+agvMeasures.front, curMapPos(2)+agvMeasures.right;                ... % front right
                    curMapPos(1)-agvMeasures.back , curMapPos(2)+agvMeasures.right;                 ...% back right
                    curMapPos(1)-agvMeasures.back , curMapPos(2)-agvMeasures.left]';                   % back left       
        % Rotate around AGV root 
        center = repmat([curMapPos(1); curMapPos(2)], 1, length(agvFrame));
        R = [cos(curMapPos(3)) -sin(curMapPos(3)); sin(curMapPos(3)) cos(curMapPos(3))];
        agvFrameRot = (R*(agvFrame - center) + center)';
    
        
        handles.agvCenter = plot(gca,curMapPos(1), curMapPos(2),'r+','LineWidth',2); hold on; grid on % AGV center
        handles.agvCoSystem = plot(gca,[curMapPos(1) agvFrameRot(1,1)], [curMapPos(2) agvFrameRot(1,2)], [curMapPos(1) agvFrameRot(2,1)], [curMapPos(2) agvFrameRot(2,2)],'k','LineWidth',1.5); % AGV coordinate system
        handles.agvBody = plot(gca,[agvFrameRot(3,1) agvFrameRot(4,1)],[agvFrameRot(3,2) agvFrameRot(4,2)],...
             [agvFrameRot(4,1) agvFrameRot(5,1)],[agvFrameRot(4,2) agvFrameRot(5,2)],...
             [agvFrameRot(5,1) agvFrameRot(6,1)],[agvFrameRot(5,2) agvFrameRot(6,2)],...
             [agvFrameRot(6,1) agvFrameRot(3,1)],[agvFrameRot(6,2) agvFrameRot(3,2)],'color',[128 128 128]./256); % AGV body   

        plot(gca, internalPosition(1), internalPosition(2), 'ro')
        
        if j == 1 
            clear handles % to keep initial footprint of AGV
        end
        xlim([-5 5])
        ylim([-5 5])
    end
    oldMapPos = curMapPos;
    mapPosBuffer(:,i) = curMapPos;
end

%%
figure
plot(mapPosBuffer(3,:))
grid on

%%
% pGainY = [0.7,0.7,0.7,0.7,0.7,0.7,0.7,0.789,0.8786,1];
% pGainA = [0.028,0.045,0.054,0.082,0.119,0.152,0.19,0.202,0.214,0.23];
% speed = [15,30,60,100,150,200,250,275,300,333.3];
% 
% figure
% plot(speed, pGainA, 'ro-'); grid on; hold on 
% plot(speed, pGainY, 'bo-')
% legend(['pGainA';'pGainY'],"Location","southeast")
% xlabel('mm/s')
