function [vl, vr] = invKinDiffDrive(v, axesParam)

    vl = v(1) - v(3) * axesParam.ringmount; % in [m/s]
    vr = v(1) + v(3) * axesParam.ringmount; % in [m/s]
    
    vl = vl / (2*pi*axesParam.diameter/2) * 60 * axesParam.gearratio; % [mm/s] to [rpm]
    vr = vr / (2*pi*axesParam.diameter/2) * 60 * axesParam.gearratio; % [mm/s] to [rpm]
    
end
