sBrakingDistance = 0.001;

if sBrakingDistance >= sTotal 
    v = v0;

elseif sBrakingDistance > (s2 + s3) % constant jerk phase 1
    guess = initialGuess;
    for n = 1:20
        guess = guess - (guess^3-(v0/(jmax/6))*guess+((sTotal-sBrakingDistance)/(jmax/6)))/(0.5*guess^2-(v0/(jmax/6)));
        value = guess^3-(v0/(jmax/6))*guess+((sTotal-sBrakingDistance)/(jmax/6));
        if abs(value) < 1e-9
            break
        end
    end
    if n > 19 
        disp('warning: no zero found')
    end

    initialGuess = guess; 
    v = v0 - 0.5 * jmax * guess^2;

elseif sBrakingDistance > s3 % constant deceleration phase
    v = sqrt(v2^2+2*amax*(sBrakingDistance-s3));        

else % constant jerk phase 2        
    v = (6^(2/3))/2*jmax^(1/3)*abs(sBrakingDistance)^(2/3);
end 

v