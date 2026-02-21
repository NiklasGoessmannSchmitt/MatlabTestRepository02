function handles = plotAGV(a,g,axesSpec) 

if strcmp(a.type,'Diff')
    % Define AGV parts
    %              X                                  Y
    agvFrame = [g.curMapPos(1)+0.1              , g.curMapPos(2);                                  ... % xAxis
                g.curMapPos(1)                  , g.curMapPos(2)+0.1;                              ... % yAxis
                g.curMapPos(1)+a.agvMeasures.front, g.curMapPos(2)-a.agvMeasures.left;                 ... % front left
                g.curMapPos(1)+a.agvMeasures.front, g.curMapPos(2)+a.agvMeasures.right;                ... % front right
                g.curMapPos(1)-a.agvMeasures.back , g.curMapPos(2)+a.agvMeasures.right;                 ...% back right
                g.curMapPos(1)-a.agvMeasures.back , g.curMapPos(2)-a.agvMeasures.left]';                   % back left       
    % Rotate around AGV root 
    center = repmat([g.curMapPos(1); g.curMapPos(2)], 1, length(agvFrame));
    R = [cos(g.curMapPos(3)) -sin(g.curMapPos(3)); sin(g.curMapPos(3)) cos(g.curMapPos(3))];
    agvFrameRot = (R*(agvFrame - center) + center)';


    handles.agvCenter = plot(axesSpec,g.curMapPos(1), g.curMapPos(2),'r+','LineWidth',2); hold on % AGV center
    handles.agvCoSystem = plot(axesSpec,[g.curMapPos(1) agvFrameRot(1,1)], [g.curMapPos(2) agvFrameRot(1,2)], [g.curMapPos(1) agvFrameRot(2,1)], [g.curMapPos(2) agvFrameRot(2,2)],'k','LineWidth',1.5); % AGV coordinate system
    handles.agvBody = plot(axesSpec,[agvFrameRot(3,1) agvFrameRot(4,1)],[agvFrameRot(3,2) agvFrameRot(4,2)],...
         [agvFrameRot(4,1) agvFrameRot(5,1)],[agvFrameRot(4,2) agvFrameRot(5,2)],...
         [agvFrameRot(5,1) agvFrameRot(6,1)],[agvFrameRot(5,2) agvFrameRot(6,2)],...
         [agvFrameRot(6,1) agvFrameRot(3,1)],[agvFrameRot(6,2) agvFrameRot(3,2)],'color',[128 128 128]./256); % AGV body   
    
elseif strcmp(a.type,'SteeredDiff_SteerMaster') || strcmp(a.type, 'SteeredDiff_DiffMaster') || ...
        strcmp(a.type, 'SteeredDiff_OneWheel')
    beta(1) = a.pivotValues(1).angularPosition;
    % Define AGV parts
    %              X                                  Y
    agvFrame = [g.curMapPos(1)+0.1                , g.curMapPos(2);                       ... % xAxis
                g.curMapPos(1)                    , g.curMapPos(2)+0.1;                   ... % yAxis
                g.curMapPos(1)+a.agvMeasures.front, g.curMapPos(2)-a.agvMeasures.left;    ... % front left
                g.curMapPos(1)+a.agvMeasures.front, g.curMapPos(2)+a.agvMeasures.right;   ... % front right
                g.curMapPos(1)-a.agvMeasures.back , g.curMapPos(2)+a.agvMeasures.right;   ... % back right
                g.curMapPos(1)-a.agvMeasures.back , g.curMapPos(2)-a.agvMeasures.left;     ... % back left
                g.curMapPos(1)+a.pivotParam(1).position.x                                      , g.curMapPos(2)+a.pivotParam(1).position.y ;                                      ... % pivot point
                g.curMapPos(1)+a.pivotParam(1).position.x-a.pivotParam(1).diameter*cos(beta(1)), g.curMapPos(2)+a.pivotParam(1).position.y-a.pivotParam(1).diameter*sin(beta(1)); ... % pivot wheel start
                g.curMapPos(1)+a.pivotParam(1).position.x+a.pivotParam(1).diameter*cos(beta(1)), g.curMapPos(2)+a.pivotParam(1).position.y+a.pivotParam(1).diameter*sin(beta(1)); ... % pivot wheel end
                ]';                          
    % Rotate around AGV root 
    center = repmat([g.curMapPos(1); g.curMapPos(2)], 1, length(agvFrame));
    R = [cos(g.curMapPos(3)) -sin(g.curMapPos(3)); sin(g.curMapPos(3)) cos(g.curMapPos(3))];
    agvFrameRot = (R*(agvFrame - center) + center)';


    handles.agvCenter = plot(axesSpec,g.curMapPos(1), g.curMapPos(2),'r+','LineWidth',2); hold on % AGV center
    handles.agvCoSystem = plot(axesSpec,[g.curMapPos(1) agvFrameRot(1,1)], [g.curMapPos(2) agvFrameRot(1,2)], [g.curMapPos(1) agvFrameRot(2,1)], [g.curMapPos(2) agvFrameRot(2,2)],'k','LineWidth',1.5); % AGV coordinate system
    handles.agvBody = plot(axesSpec,[agvFrameRot(3,1) agvFrameRot(4,1)],[agvFrameRot(3,2) agvFrameRot(4,2)],...
          [agvFrameRot(4,1) agvFrameRot(5,1)],[agvFrameRot(4,2) agvFrameRot(5,2)],...
          [agvFrameRot(5,1) agvFrameRot(6,1)],[agvFrameRot(5,2) agvFrameRot(6,2)],... 
          [agvFrameRot(6,1) agvFrameRot(3,1)],[agvFrameRot(6,2) agvFrameRot(3,2)],'color',[128 128 128]./256); % AGV body    
    handles.pivotCenter = plot(axesSpec,agvFrameRot(7,1),agvFrameRot(7,2),'bo','LineWidth',2); % pivot center
    handles.pivotWheel = plot(axesSpec,[agvFrameRot(8,1) agvFrameRot(9,1)],[agvFrameRot(8,2) agvFrameRot(9,2)],'b','LineWidth',1.5); % pivot wheel

elseif strcmp(a.type, 'Tricycle') || strcmp(a.type, 'TricycleRear') 
    beta(1) = a.pivotValues(1).angularPosition;
    % Define AGV parts
    %              X                                                                             Y
    agvFrame = [g.curMapPos(1)+0.1                                                          , g.curMapPos(2);                                    ... % xAxis
                g.curMapPos(1)                                                              , g.curMapPos(2)+0.1;                                ... % yAxis
                g.curMapPos(1)+a.pivotParam(1).position.x                                   , g.curMapPos(2)+a.pivotParam(1).position.y ;        ... % pivot point
                g.curMapPos(1)+a.pivotParam(1).position.x-a.pivotParam(1).diameter*cos(beta(1)), g.curMapPos(2)+a.pivotParam(1).position.y-a.pivotParam(1).diameter*sin(beta(1)); ... % pivot wheel start
                g.curMapPos(1)+a.pivotParam(1).position.x+a.pivotParam(1).diameter*cos(beta(1)), g.curMapPos(2)+a.pivotParam(1).position.y+a.pivotParam(1).diameter*sin(beta(1)); ... % pivot wheel end
                g.curMapPos(1)+a.agvMeasures.front                                          , g.curMapPos(2)-a.agvMeasures.left;                 ... % front left
                g.curMapPos(1)+a.agvMeasures.front                                          , g.curMapPos(2)+a.agvMeasures.right;                ... % front right
                g.curMapPos(1)-a.agvMeasures.back                                           , g.curMapPos(2)+a.agvMeasures.right;                ... % back right
                g.curMapPos(1)-a.agvMeasures.back                                           , g.curMapPos(2)-a.agvMeasures.left]';                   % back left    

    % Rotate around AGV root 
    center = repmat([g.curMapPos(1); g.curMapPos(2)], 1, length(agvFrame));
    R = [cos(g.curMapPos(3)) -sin(g.curMapPos(3)); sin(g.curMapPos(3)) cos(g.curMapPos(3))];
    agvFrameRot = (R*(agvFrame - center) + center)';

    handles.agvCenter = plot(axesSpec,g.curMapPos(1), g.curMapPos(2),'r+','LineWidth',2); hold on % AGV center
    handles.agvCoSystem = plot(axesSpec,[g.curMapPos(1) agvFrameRot(1,1)], [g.curMapPos(2) agvFrameRot(1,2)], [g.curMapPos(1) agvFrameRot(2,1)], [g.curMapPos(2) agvFrameRot(2,2)],'k','LineWidth',1.5); % AGV coordinate system
    handles.pivotCenter = plot(axesSpec,agvFrameRot(3,1),agvFrameRot(3,2),'bo','LineWidth',2); % pivot center
    handles.pivotWheel = plot(axesSpec,[agvFrameRot(4,1) agvFrameRot(5,1)],[agvFrameRot(4,2) agvFrameRot(5,2)],'b','LineWidth',1.5); % pivot wheel
    handles.agvBody = plot(axesSpec,[agvFrameRot(6,1) agvFrameRot(7,1)],[agvFrameRot(6,2) agvFrameRot(7,2)],...
         [agvFrameRot(7,1) agvFrameRot(8,1)],[agvFrameRot(7,2) agvFrameRot(8,2)],...
         [agvFrameRot(8,1) agvFrameRot(9,1)],[agvFrameRot(8,2) agvFrameRot(9,2)],...
         [agvFrameRot(9,1) agvFrameRot(6,1)],[agvFrameRot(9,2) agvFrameRot(6,2)],'color',[128 128 128]./256); % AGV body    
    

%  elseif strcmp(a.type, 'TricycleRear') 
%     beta(1) = a.pivotValues(1).angularPosition;
%     % Define AGV parts
%     %              X                                                                             Y
%     agvFrame = [g.curMapPos(1)+0.1                                                          , g.curMapPos(2);                                    ... % xAxis
%                 g.curMapPos(1)                                                              , g.curMapPos(2)+0.1;                                ... % yAxis
%                 g.curMapPos(1)+a.pivotParam(1).position.x                                   , g.curMapPos(2)+a.pivotParam(1).position.y ;        ... % pivot point
%                 g.curMapPos(1)+a.pivotParam(1).position.x-a.pivotParam(1).diameter*cos(beta(1)), g.curMapPos(2)+a.pivotParam(1).position.y-a.pivotParam(1).diameter*sin(beta(1)); ... % pivot wheel start
%                 g.curMapPos(1)+a.pivotParam(1).position.x+a.pivotParam(1).diameter*cos(beta(1)), g.curMapPos(2)+a.pivotParam(1).position.y+a.pivotParam(1).diameter*sin(beta(1)); ... % pivot wheel end
%                 g.curMapPos(1)+a.agvMeasures.front                                          , g.curMapPos(2)-a.agvMeasures.left;                 ... % front left
%                 g.curMapPos(1)+a.agvMeasures.front                                          , g.curMapPos(2)+a.agvMeasures.right;                ... % front right
%                 g.curMapPos(1)-a.agvMeasures.back                                           , g.curMapPos(2)+a.agvMeasures.right;                ... % back right
%                 g.curMapPos(1)-a.agvMeasures.back                                           , g.curMapPos(2)-a.agvMeasures.left]';                   % back left    
% 
%     % Rotate around AGV root 
%     center = repmat([g.curMapPos(1); g.curMapPos(2)], 1, length(agvFrame));
%     R = [cos(g.curMapPos(3)) -sin(g.curMapPos(3)); sin(g.curMapPos(3)) cos(g.curMapPos(3))];
%     agvFrameRot = (R*(agvFrame - center) + center)';
% 
%     plot(g.curMapPos(1), g.curMapPos(2),'r+','LineWidth',2); hold on % AGV center
%     plot([g.curMapPos(1) agvFrameRot(1,1)], [g.curMapPos(2) agvFrameRot(1,2)],'k','LineWidth',1.5) % AGV coordinate system (X)
%     plot([g.curMapPos(1) agvFrameRot(2,1)], [g.curMapPos(2) agvFrameRot(2,2)],'k','LineWidth',1.5) % AGV coordinate system (Y)
%     plot(agvFrameRot(3,1),agvFrameRot(3,2),'bo','LineWidth',2) % pivot center
%     plot([agvFrameRot(4,1) agvFrameRot(5,1)],[agvFrameRot(4,2) agvFrameRot(5,2)],'b','LineWidth',1.5) % pivot wheel
%     plot([agvFrameRot(6,1) agvFrameRot(7,1)],[agvFrameRot(6,2) agvFrameRot(7,2)],'b') % AGV frame
%     plot([agvFrameRot(7,1) agvFrameRot(8,1)],[agvFrameRot(7,2) agvFrameRot(8,2)],'b') % AGV frame
%     plot([agvFrameRot(8,1) agvFrameRot(9,1)],[agvFrameRot(8,2) agvFrameRot(9,2)],'b') % AGV frame
%     plot([agvFrameRot(9,1) agvFrameRot(6,1)],[agvFrameRot(9,2) agvFrameRot(6,2)],'b') % AGV frame  
    
elseif strcmp(a.type, 'Mecanum')
    % Define AGV parts
    %              X                                                          Y
    agvFrame = [g.curMapPos(1)+0.1                     , g.curMapPos(2);                          ... % xAxis
                g.curMapPos(1)                         , g.curMapPos(2)+0.1;                      ... % yAxis
                g.curMapPos(1)+a.pivotParam(1).position.x, g.curMapPos(2)+a.pivotParam(1).position.y; ... % wheel FR
                g.curMapPos(1)+a.pivotParam(2).position.x, g.curMapPos(2)+a.pivotParam(2).position.y; ... % wheel FL
                g.curMapPos(1)+a.pivotParam(3).position.x, g.curMapPos(2)+a.pivotParam(3).position.y; ... % wheel BR
                g.curMapPos(1)+a.pivotParam(4).position.x, g.curMapPos(2)+a.pivotParam(4).position.y; ... % wheel BL
                g.curMapPos(1)+a.agvMeasures.front       , g.curMapPos(2)-a.agvMeasures.left;         ... % front left
                g.curMapPos(1)+a.agvMeasures.front       , g.curMapPos(2)+a.agvMeasures.right;        ... % front right
                g.curMapPos(1)-a.agvMeasures.back        , g.curMapPos(2)+a.agvMeasures.right;        ... % back right
                g.curMapPos(1)-a.agvMeasures.back        , g.curMapPos(2)-a.agvMeasures.left]';           % back left       
    % Rotate around AGV root 
    center = repmat([g.curMapPos(1); g.curMapPos(2)], 1, length(agvFrame));
    R = [cos(g.curMapPos(3)) -sin(g.curMapPos(3)); sin(g.curMapPos(3)) cos(g.curMapPos(3))];
    agvFrameRot = (R*(agvFrame - center) + center)';

    handles.agvCenter = plot(axesSpec,g.curMapPos(1), g.curMapPos(2),'r+','LineWidth',2); hold on % AGV center
    handles.agvCoSystem = plot(axesSpec,[g.curMapPos(1) agvFrameRot(1,1)], [g.curMapPos(2) agvFrameRot(1,2)], [g.curMapPos(1) agvFrameRot(2,1)], [g.curMapPos(2) agvFrameRot(2,2)],'k','LineWidth',1.5); % AGV coordinate system
    handles.pivotWheel(1) = plot(axesSpec,agvFrameRot(3,1),agvFrameRot(3,2),'bo','LineWidth',2); % wheel FR
    handles.pivotWheel(2) = plot(axesSpec,agvFrameRot(4,1),agvFrameRot(4,2),'bo','LineWidth',2); % wheel FL
    handles.pivotWheel(3) = plot(axesSpec,agvFrameRot(5,1),agvFrameRot(5,2),'bo','LineWidth',2); % wheel BR
    handles.pivotWheel(4) = plot(axesSpec,agvFrameRot(6,1),agvFrameRot(6,2),'bo','LineWidth',2); % wheel BL
    handles.agvBody = plot(axesSpec,[agvFrameRot( 7,1) agvFrameRot( 8,1)],[agvFrameRot( 7,2) agvFrameRot( 8,2)], ... 
                           [agvFrameRot( 8,1) agvFrameRot( 9,1)],[agvFrameRot( 8,2) agvFrameRot( 9,2)],... 
                           [agvFrameRot( 9,1) agvFrameRot(10,1)],[agvFrameRot( 9,2) agvFrameRot(10,2)],... 
                           [agvFrameRot(10,1) agvFrameRot( 7,1)],[agvFrameRot(10,2) agvFrameRot( 7,2)],'color',[128 128 128]./256); % AGV body  

elseif ( strcmp(a.type, 'PivotActive') && a.n == 2 ) || ( strcmp(a.type, 'PivotPassive') && a.n == 2 ) 
        beta(1) = a.pivotValues(1).angularPosition;
        beta(2) = a.pivotValues(2).angularPosition;
        % Define AGV parts
        %              X                                                                         Y
        agvFrame = [g.curMapPos(1)+0.1                                                         , g.curMapPos(2);                                                              ... % xAxis
                    g.curMapPos(1)                                                             , g.curMapPos(2)+0.1;                                                          ... % yAxis
                    g.curMapPos(1)+a.pivotParam(1).position.x                                    , g.curMapPos(2)+a.pivotParam(1).position.y;                                     ... % pivot point(1)
                    g.curMapPos(1)+a.pivotParam(1).position.x-a.pivotParam(1).diameter*cos(beta(1)), g.curMapPos(2)+a.pivotParam(1).position.y-a.pivotParam(1).diameter*sin(beta(1)); ... % pivot wheel start
                    g.curMapPos(1)+a.pivotParam(1).position.x+a.pivotParam(1).diameter*cos(beta(1)), g.curMapPos(2)+a.pivotParam(1).position.y+a.pivotParam(1).diameter*sin(beta(1)); ... % pivot wheel end
                    g.curMapPos(1)+a.pivotParam(2).position.x                                    , g.curMapPos(2)+a.pivotParam(2).position.y;                                     ... % pivot point(2)
                    g.curMapPos(1)+a.pivotParam(2).position.x-a.pivotParam(1).diameter*cos(beta(2)), g.curMapPos(2)+a.pivotParam(2).position.y-a.pivotParam(2).diameter*sin(beta(2)); ... % pivot wheel start
                    g.curMapPos(1)+a.pivotParam(2).position.x+a.pivotParam(1).diameter*cos(beta(2)), g.curMapPos(2)+a.pivotParam(2).position.y+a.pivotParam(2).diameter*sin(beta(2)); ... % pivot wheel end
                    g.curMapPos(1)+a.agvMeasures.front                                           , g.curMapPos(2)-a.agvMeasures.left;                                             ... % front left
                    g.curMapPos(1)+a.agvMeasures.front                                           , g.curMapPos(2)+a.agvMeasures.right;                                            ... % front right
                    g.curMapPos(1)-a.agvMeasures.back                                            , g.curMapPos(2)+a.agvMeasures.right;                                            ... % back right
                    g.curMapPos(1)-a.agvMeasures.back                                            , g.curMapPos(2)-a.agvMeasures.left;                                             ... % back left
                    ]';                                                     
        % Rotate around AGV root 
        center = repmat([g.curMapPos(1); g.curMapPos(2)], 1, length(agvFrame));
        R = [cos(g.curMapPos(3)) -sin(g.curMapPos(3)); sin(g.curMapPos(3)) cos(g.curMapPos(3))];
        agvFrameRot = (R*(agvFrame - center) + center)';

        handles.agvCenter = plot(axesSpec,g.curMapPos(1), g.curMapPos(2),'r+','LineWidth',2); hold on % AGV center
        handles.agvCoSystem = plot(axesSpec,[g.curMapPos(1) agvFrameRot(1,1)], [g.curMapPos(2) agvFrameRot(1,2)], [g.curMapPos(1) agvFrameRot(2,1)], [g.curMapPos(2) agvFrameRot(2,2)],'k','LineWidth',1.5); % AGV coordinate system
        handles.pivotCenter(1) = plot(axesSpec,agvFrameRot(3,1),agvFrameRot(3,2),'bo','LineWidth',2); % pivot center
        handles.pivotCenter(2) = plot(axesSpec,agvFrameRot(6,1),agvFrameRot(6,2),'bo','LineWidth',2); % pivot center
        handles.pivotWheel(1) = plot(axesSpec,[agvFrameRot(4,1) agvFrameRot(5,1)],[agvFrameRot(4,2) agvFrameRot(5,2)],'b','LineWidth',1.5); % pivot wheel
        handles.pivotWheel(2) = plot(axesSpec,[agvFrameRot(7,1) agvFrameRot(8,1)],[agvFrameRot(7,2) agvFrameRot(8,2)],'b','LineWidth',1.5); % pivot wheel
        handles.agvBody = plot(axesSpec,[agvFrameRot(9,1) agvFrameRot(10,1)],[agvFrameRot(9,2) agvFrameRot(10,2)], ... 
                               [agvFrameRot(10,1) agvFrameRot(11,1)],[agvFrameRot(10,2) agvFrameRot(11,2)],... 
                               [agvFrameRot(11,1) agvFrameRot(12,1)],[agvFrameRot(11,2) agvFrameRot(12,2)],... 
                               [agvFrameRot(12,1) agvFrameRot(9,1)],[agvFrameRot(12,2) agvFrameRot(9,2)],'color',[128 128 128]./256); % AGV body     

elseif strcmp(a.type, 'PivotActive') && a.n == 3
    
        beta(1) = a.pivotValues(1).angularPosition;
        beta(2) = a.pivotValues(2).angularPosition;
        beta(3) = a.pivotValues(3).angularPosition;
        % Define AGV parts
        %              X                                                                         Y
        agvFrame = [g.curMapPos(1)+0.1                                                         , g.curMapPos(2);                                                              ... % xAxis
                    g.curMapPos(1)                                                             , g.curMapPos(2)+0.1;                                                          ... % yAxis
                    g.curMapPos(1)+a.pivotParam(1).position.x                                    , g.curMapPos(2)+a.pivotParam(1).position.y;                                     ... % pivot point(1)
                    g.curMapPos(1)+a.pivotParam(1).position.x-a.pivotParam(1).diameter*cos(beta(1)), g.curMapPos(2)+a.pivotParam(1).position.y-a.pivotParam(1).diameter*sin(beta(1)); ... % pivot wheel start
                    g.curMapPos(1)+a.pivotParam(1).position.x+a.pivotParam(1).diameter*cos(beta(1)), g.curMapPos(2)+a.pivotParam(1).position.y+a.pivotParam(1).diameter*sin(beta(1)); ... % pivot wheel end
                    g.curMapPos(1)+a.pivotParam(2).position.x                                    , g.curMapPos(2)+a.pivotParam(2).position.y;                                     ... % pivot point(2)
                    g.curMapPos(1)+a.pivotParam(2).position.x-a.pivotParam(1).diameter*cos(beta(2)), g.curMapPos(2)+a.pivotParam(2).position.y-a.pivotParam(2).diameter*sin(beta(2)); ... % pivot wheel start
                    g.curMapPos(1)+a.pivotParam(2).position.x+a.pivotParam(1).diameter*cos(beta(2)), g.curMapPos(2)+a.pivotParam(2).position.y+a.pivotParam(2).diameter*sin(beta(2)); ... % pivot wheel end
                    g.curMapPos(1)+a.agvMeasures.front                                         , g.curMapPos(2)-a.agvMeasures.left;                                             ... % front left
                    g.curMapPos(1)+a.agvMeasures.front                                         , g.curMapPos(2)+a.agvMeasures.right;                                            ... % front right
                    g.curMapPos(1)-a.agvMeasures.back                                          , g.curMapPos(2)+a.agvMeasures.right;                                            ... % back right
                    g.curMapPos(1)-a.agvMeasures.back                                          , g.curMapPos(2)-a.agvMeasures.left;                                             ... % back left
                    g.curMapPos(1)+a.pivotParam(3).position.x                                    , g.curMapPos(2)+a.pivotParam(3).position.y;                                     ... % pivot point(3)
                    g.curMapPos(1)+a.pivotParam(3).position.x-a.pivotParam(1).diameter*cos(beta(3)), g.curMapPos(2)+a.pivotParam(3).position.y-a.pivotParam(2).diameter*sin(beta(3)); ... % pivot wheel start
                    g.curMapPos(1)+a.pivotParam(3).position.x+a.pivotParam(1).diameter*cos(beta(3)), g.curMapPos(2)+a.pivotParam(3).position.y+a.pivotParam(2).diameter*sin(beta(3)); ... % pivot wheel end
                    ]';                                               % back left       
        % Rotate around AGV root 
        center = repmat([g.curMapPos(1); g.curMapPos(2)], 1, length(agvFrame));
        R = [cos(g.curMapPos(3)) -sin(g.curMapPos(3)); sin(g.curMapPos(3)) cos(g.curMapPos(3))];
        agvFrameRot = (R*(agvFrame - center) + center)';

        handles.agvCenter = plot(axesSpec,g.curMapPos(1), g.curMapPos(2),'r+','LineWidth',2); hold on % AGV center
        handles.agvCoSystem = plot(axesSpec,[g.curMapPos(1) agvFrameRot(1,1)], [g.curMapPos(2) agvFrameRot(1,2)], [g.curMapPos(1) agvFrameRot(2,1)], [g.curMapPos(2) agvFrameRot(2,2)],'k','LineWidth',1.5); % AGV coordinate system
        handles.pivotCenter(1) = plot(axesSpec,agvFrameRot( 3,1),agvFrameRot( 3,2),'bo','LineWidth',2); % pivot center
        handles.pivotCenter(2) = plot(axesSpec,agvFrameRot( 6,1),agvFrameRot( 6,2),'bo','LineWidth',2); % pivot center
        handles.pivotCenter(3) = plot(axesSpec,agvFrameRot(13,1),agvFrameRot(13,2),'bo','LineWidth',2); % pivot center
        handles.pivotWheel(1) = plot(axesSpec,[agvFrameRot( 4,1) agvFrameRot( 5,1)],[agvFrameRot( 4,2) agvFrameRot( 5,2)],'b','LineWidth',1.5); % pivot wheel
        handles.pivotWheel(2) = plot(axesSpec,[agvFrameRot( 7,1) agvFrameRot( 8,1)],[agvFrameRot( 7,2) agvFrameRot( 8,2)],'b','LineWidth',1.5); % pivot wheel       
        handles.pivotWheel(3) = plot(axesSpec,[agvFrameRot(14,1) agvFrameRot(15,1)],[agvFrameRot(14,2) agvFrameRot(15,2)],'b','LineWidth',1.5); % pivot wheel

        handles.agvBody = plot(axesSpec,[agvFrameRot( 9,1) agvFrameRot(10,1)],[agvFrameRot( 9,2) agvFrameRot(10,2)],... 
                               [agvFrameRot(10,1) agvFrameRot(11,1)],[agvFrameRot(10,2) agvFrameRot(11,2)],...
                               [agvFrameRot(11,1) agvFrameRot(12,1)],[agvFrameRot(11,2) agvFrameRot(12,2)],... 
                               [agvFrameRot(12,1) agvFrameRot( 9,1)],[agvFrameRot(12,2) agvFrameRot( 9,2)],'color',[128 128 128]./256); % AGV body  

        
else
   disp('Unknown AGV type') 
end