function plotXYAVelocities(g,i)

errorIndex = 0; 
positioningIndex = i; 

figure('units','normalized','outerposition',[0 0 1 1])
subplot(311) 
plot(1:i, g.vSetBuffer(1,1:i)); hold on; grid on 
plot(1:i, g.vRampedBuffer(1,1:i))
plot(1:i, g.vCurrentBuffer(1,1:i))
ylim = get(gca,'ylim');
if errorIndex > 0; plot([errorIndex errorIndex],[ylim(1) ylim(2)],'k-.','LineWidth',1); end
h = fill([positioningIndex,i,i,positioningIndex],...
         [ylim(1),ylim(1),ylim(2),ylim(2)],'green');
set(h,'Linestyle','none','facealpha',.3)
title('v_X')
if errorIndex > 0; legend('vSetpoint_X','vRamped_X','Error','Positioning'); 
else; legend('vSetpoint_X','vRamped_X','Positioning'); end
    
subplot(312)
plot(1:i, g.vSetBuffer(2,1:i)); hold on; grid on
plot(1:i, g.vRampedBuffer(2,1:i))
plot(1:i, g.vCurrentBuffer(2,1:i))
ylim = get(gca,'ylim');
if errorIndex > 0; plot([errorIndex errorIndex],[ylim(1) ylim(2)],'k-.','LineWidth',1); end
h = fill([positioningIndex,i,i,positioningIndex],...
         [ylim(1),ylim(1),ylim(2),ylim(2)],'green');
set(h,'Linestyle','none','facealpha',.3)
title('v_Y')
if errorIndex > 0; legend('vSetpoint_Y','vRamped_Y','Error','Positioning');
else; legend('vSetpoint_Y','vRamped_Y','Positioning'); end

subplot(313) 
plot(1:i, g.vSetBuffer(3,1:i)); hold on; grid on
plot(1:i, g.vRampedBuffer(3,1:i))
plot(1:i, g.vCurrentBuffer(3,1:i))
ylim = get(gca,'ylim');
if errorIndex > 0; plot([errorIndex errorIndex],[ylim(1) ylim(2)],'k-.','LineWidth',1); end
h = fill([positioningIndex,i,i,positioningIndex],...
         [ylim(1),ylim(1),ylim(2),ylim(2)],'green');
set(h,'Linestyle','none','facealpha',.3)
title('v_A')
if errorIndex > 0; legend('vSetpoint_A','vRamped_A','Error','Positioning');
else; legend('vSetpoint_A','vRamped_A','Positioning'); end