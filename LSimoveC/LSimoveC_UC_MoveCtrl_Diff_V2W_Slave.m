function [vl,vr] = LSimoveC_UC_MoveCtrl_Diff_V2W_Slave(vs, alpha, pivotParam)

    r = pivotParam(1).position.x * tan(pi/2 - alpha);
    va = vs/pivotParam(1).position.x * sin(alpha); 
    
    vr = va * (r + pivotParam(2).ringmount) / (2.0*pi) / (pivotParam(2).diameter/2.0) * 60.0 * pivotParam(2).gearratio;
    vl = va * (r - pivotParam(2).ringmount) / (2.0*pi) / (pivotParam(2).diameter/2.0) * 60.0 * pivotParam(2).gearratio;

end