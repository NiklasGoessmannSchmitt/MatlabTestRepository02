function [v] = fwdKinDiffDrive (vl, vr, axesParam)

    v(1) = (vl + vr) / 2.0; % in [rpm]
    v(2) = zeros(1,length(vl));
    v(3) = (vr - vl) / (2.0 * axesParam.ringmount); % in [rpm]
    
    v(1) = v(1) * 2*pi * axesParam.diameter/2.0 / 60.0 / axesParam.gearratio; % in [m/s]
    v(3) = v(3) * 2*pi * axesParam.diameter/2.0 / 60.0 / axesParam.gearratio; % in [m/s]
end