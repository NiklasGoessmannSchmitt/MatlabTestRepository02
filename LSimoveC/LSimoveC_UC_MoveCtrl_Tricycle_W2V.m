function vCurrent = LSimoveC_UC_MoveCtrl_Tricycle_W2V(vs, alpha, pivotParam) 

CONST_NULL_P = 1e-5;

tempAlpha = alpha; 
tempVs = vs;

if pivotParam.position.y ~= 0 && abs(tempAlpha) > CONST_NULL_P 
    
    tempAlpha = atan(pivotParam.position.x / (pivotParam.position.x / tan(alpha) + pivotParam.position.y) ); 
    
    if pivotParam.position.y < 0 
        if tempAlpha < atan(pivotParam.position.x / pivotParam.position.y) 
            tempAlpha = tempAlpha - pi; 
        end
    elseif pivotParam.position.y > 0 
        if tempAlpha > atan(pivotParam.position.x / pivotParam.position.y) 
            tempAlpha = tempAlpha + pi; 
        end
    end 
    
    tempVs = vs * sin(alpha) / sin(tempAlpha); 
end

    
    
    vCurrent(1) = vs * cos(tempAlpha); 
    vCurrent(2) = 0; 
    vCurrent(3) = tempVs / pivotParam.position.x * sin(tempAlpha); 

end