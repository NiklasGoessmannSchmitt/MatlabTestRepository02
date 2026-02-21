function f = axisframe(xyData)

f = [min([-1 min(xyData(1,:))*1.1]) ... % x_min
     max([ 1 max(xyData(1,:))*1.1]) ... % x_max
     min([-1 min(xyData(2,:))*1.1]) ... % y_min
     max([ 1 max(xyData(2,:))*1.1])];   % y_max

end