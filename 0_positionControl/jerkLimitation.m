close all
clear all
clc
%%
basicPlotsOnly = false; 
setpointDelay = 100; % ms

decelerationPercentage = 0.8; 
v0 = 1.0; % [m/s] 
jmax = 0.36; % [m/s^3] 
amax = 0.7 * decelerationPercentage; % [m/s^2]
v3 = 0.1; % [m/s] -- vMin 

brakingReserve = 0.005; % m; 
% gets substracted from the actual braking distance prior to the calculation 
% of the jerk algorithm
% when used with v3 = 0.0, then it should also be 0. Otherwise, the vehicle
% stops just that amount short 
if v3 == 0
    brakingReserve = 0; 
end

aReachable = jmax*sqrt((v0-v3)/jmax);
if aReachable < amax
    fprintf('The maximum reachable deceleration was adjusted from %.5f m/s^2 to %.5f m/s^2. \n', amax, aReachable)
end
amax = min(amax,aReachable);

v1 = v0 - (amax^2/(2*jmax)); 
v2 = (amax^2/(2*jmax)) + v3; 
s1 = v0*amax/jmax - 1/6*jmax*(amax/jmax)^3;
s2 = (v1 - v2)^2/(2*amax) + v2*(v1 - v2)/amax;
s3 = 1/6*jmax*(amax/jmax)^3 + v3*(amax/jmax);

sTotal = s1+s2+s3;
fprintf('The breaking distance to get from v0: %.3f m/s to v3: %.3f m/s is s: %.5f m. \n', v0, v3, sTotal)

%%
t = 1e-3;
sBrakingDistancePhysical = sTotal + 0.2; 
sBrakingDistanceCalc = sBrakingDistancePhysical - brakingReserve;

vSetpointBuffer = zeros(1,5000);
vActualBuffer = vSetpointBuffer;
aBuffer = vSetpointBuffer; 
sBuffer = aBuffer; 

t_initialGuess = 0.0; 
nMax = 0; 
n = 0;
zeroTolerance = 1e-9;
jerkPhase = 0; 

for i = 1:10000  

    if sBrakingDistanceCalc >= sTotal 
        vSetpoint = v0;
        enterConstJerk1 = i; 

    elseif sBrakingDistanceCalc > (s2 + s3) % constant jerk phase 1
        if jerkPhase ~= 1 
            t_guess = t_initialGuess;
            jerkPhase = 1; 
        end

        for n = 1:20
            sTraveledInJerkPhaseOne = sTotal-sBrakingDistanceCalc;
            t_guess = t_guess - (t_guess^3-(v0/(jmax/6))*t_guess+((sTraveledInJerkPhaseOne)/(jmax/6)))/(3*t_guess^2-(v0/(jmax/6)));
            zero_value =         t_guess^3-(v0/(jmax/6))*t_guess+((sTraveledInJerkPhaseOne)/(jmax/6));
            if abs(zero_value) < zeroTolerance
                break
            elseif n == 20
                fprintf('warning: no zero found. Last value was %.12f. \n', zero_value)
            end
        end
        vSetpoint = v0 - 0.5*jmax * t_guess^2;
        
    elseif sBrakingDistanceCalc > s3 % constant deceleration phase
        vSetpoint = sqrt(v2^2+2*amax*(sBrakingDistanceCalc-s3));        

    elseif sBrakingDistanceCalc > 0  % constant jerk phase 2        

        if v3 == 0.0 
            vSetpoint = (6^(2/3))/2*jmax^(1/3)*abs(sBrakingDistanceCalc)^(2/3);
        else
            if jerkPhase ~= 2 
                t_guess = t_initialGuess;
                jerkPhase = 2; 
            end
                   
            for n = 1:20  
                sTraveledInJerkPhaseTwo = s3-sBrakingDistanceCalc;
                t_guess = t_guess - (t_guess^3+((3*-amax)/jmax)*t_guess^2+((6*v2)/jmax)*t_guess-(6*sTraveledInJerkPhaseTwo)/jmax)/(3*t_guess^2+2*((3*-amax)/jmax)*t_guess+((6*v2)/jmax));
                zero_value =         t_guess^3+((3*-amax)/jmax)*t_guess^2+((6*v2)/jmax)*t_guess-(6*sTraveledInJerkPhaseTwo)/jmax;
                if abs(zero_value) < zeroTolerance  
                    break  
                elseif n == 20  
                    fprintf('warning: no zero found. Last value was %.12f. \n', zero_value)  
                end  
            end  
            vSetpoint = v2 - amax*t_guess + 0.5*jmax*t_guess^2; 
        end
        exitConstJerk2 = i; 
    else
        vSetpoint = v3; 
    end 

    if ~isreal(vSetpoint)
        disp('velocity setpoint non-real -- exit')
        break
    end

    vSetpointBuffer(i) = vSetpoint; 

    if i > setpointDelay
        vActual = vSetpointBuffer(i-setpointDelay); 
    else
        vActual = vSetpoint;
    end 
    sBrakingDistancePhysical = sBrakingDistancePhysical - vActual*t*1.06;
    sBrakingDistanceCalc = sBrakingDistancePhysical - brakingReserve;

    vActualBuffer(i) = vActual; 
    sBuffer(i) = sBrakingDistancePhysical;

    nMax = max([nMax,n]);
    if sBrakingDistancePhysical <= 1e-3 % || v <= v3
        break
    end
    sBrakingDistanceOld = sBrakingDistancePhysical; 
end

fprintf('... and took %.3f seconds. \n', (exitConstJerk2-enterConstJerk1)/1000)
fprintf('The maximum iterationsteps at a tolerance of %.1fe-9 of the Newton method was %.0f. \n', zeroTolerance/1e-9, nMax)
fprintf('Due to a setpoint delay of %.1f ms, the maximum commanded deceleration was %.3f m/s^2 despite the set maximum of %.3f m/s^2. \n', setpointDelay, max(abs(diff(vActualBuffer(1:i-1))./t)), abs(amax))
fprintf('This would have required a deceleration percentage of no higher than %.2f --> %.3f m/s^2 * %.2f = %.3f m/s^2. \n', 1/(max(abs(diff(vActualBuffer(1:i-1))./t))/abs(amax)), max(abs(diff(vActualBuffer(1:i-1))./t)), 1/(max(abs(diff(vActualBuffer(1:i-1))./t))/abs(amax)), abs(amax))

%%
figure
subplot(411)
plot(diff(diff(vSetpointBuffer(1:i-1))./t)./t); grid on; hold on
plot(diff(diff(vActualBuffer(1:i-1))./t)./t)
xline(exitConstJerk2)
ylim([-jmax-1 jmax+1])
legend('setpoint','actual')
title('jerk')

subplot(412)
plot(diff(vSetpointBuffer(1:i-1))./t); grid on; hold on
plot(diff(vActualBuffer(1:i-1))./t);
xline(exitConstJerk2)
yline(-amax)
ylim([-amax-0.1 0])
legend('setpoint','actual','amax')
title('deceleration')

subplot(413)
plot(vSetpointBuffer(1:i-1)); grid on; hold on
plot(vActualBuffer(1:i-1))
xline(exitConstJerk2)
ylim([0 max(vSetpointBuffer(1:i-1)+0.1)])
legend('setpoint','actual')
title('velocity')

subplot(414)
plot(sBuffer(1:i-1)); grid on
xline(exitConstJerk2)
title('distance')

%%
if false 
    figure 
    plot(vActualBuffer(1:i))
    hold on 
    plot(diff(vActualBuffer(1:i))./t);
    
    figure 
    plot(vSetpointBuffer(1:i))
    hold on 
    plot(diff(vSetpointBuffer(1:i))./t);
end 

if ~ basicPlotsOnly 
    figure 
    plot(sBuffer(1:i),vSetpointBuffer(1:i),'LineWidth',1.5); grid on; hold on
    plot(sBuffer(1:i),vActualBuffer(1:i),'LineWidth',1.5);
    xlabel('brakingDistance [m]')
    ylabel('velocity [m/s]')
    legend('setpoint','actual')
    
    % RV specifics: 
    yline(0.025) % minimum velocity = 0.025
    xline(0.005) % positioning tolerance = 0.005
    xlim([0 0.01])
    ylim([0 0.05])
    
    figure 
    plot(sBuffer(1:i-1),diff(vSetpointBuffer(1:i))./t,'LineWidth',1.5)
    grid on
    xlabel('brakingDistance [m]')
    ylabel('deceleration [m/s^2]')
end 