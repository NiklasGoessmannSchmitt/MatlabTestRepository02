function v = XA_Pilot(carrot, interimTargetLength, pushLength, betaSetpoint, k_phi_only_range, limit_steer_range, targetSpeed, finalTargetLength)
%% persistent variables (static) 
persistent stat_k_phi
persistent stat_k_alpha
persistent stat_k_roh
persistent stat_phi_old
persistent stat_roh_old
persistent statDistanceSetpoint_old
persistent locked 
if isempty(stat_k_phi)
    stat_k_phi = 0;
    stat_k_alpha = 0;
    stat_k_roh = 0;
    stat_phi_old = 0; 
    stat_roh_old = 0;
    statDistanceSetpoint_old = zeros(3,1);
    locked = false; 
end

%%
k_rot   = 1;
% k_phi_only_range = 0.2;
k_alpha =  2.50;
k_phi   = -0.75; 
k_roh   =  0.45; 

%% INIT
distanceDeviation = carrot - statDistanceSetpoint_old; 
if ( sqrt( distanceDeviation(1)^2 + distanceDeviation(2)^2 ) > 0.1 ... % [m]
    || abs(distanceDeviation(3) ) > 0.1 ) %% [rad]
    
    stat_k_phi = k_phi;
    stat_k_alpha = k_alpha;
    stat_k_roh = k_roh;
    locked = false; 
end
statDistanceSetpoint_old = carrot;

%%
stat_roh = sqrt( carrot(1)^2 + carrot(2)^2 );
stat_phi = carrot(3);

if carrot(1) ~= 0 
    stat_alpha = atan( carrot(2) / carrot(1) );
else
    stat_alpha = 0.0; 
end

%% adjust the control parameters
if stat_roh < k_phi_only_range && k_phi_only_range > 0 && stat_k_alpha ~= 0 
    
    temp_a = stat_k_alpha * stat_alpha + stat_k_phi * stat_phi;
    if k_rot > 0 
        stat_k_phi = k_rot;
    else
        if abs(stat_phi) < 0.0005  % to avoid numerical errors
            stat_k_phi = 0;
        else
            stat_k_phi = max( [1, abs(temp_a /stat_phi)] );
        end
    end   
    stat_k_alpha = 0.0; 
end

stat_phi_old = stat_phi;
stat_roh_old = stat_roh;

if stat_roh < limit_steer_range && limit_steer_range > 0 
%% stat_roh und stat_phi brute force --> streng monoton fallend

    if abs(stat_phi) > abs(stat_phi_old) 
        stat_phi = stat_phi_old;
    else
        stat_phi_old = stat_phi;
    end

    if abs(stat_roh) > abs(stat_roh_old)
        stat_roh = stat_roh_old;
    else
        stat_roh_old = stat_roh;
    end  
end

%% assemble the velocity vector
if carrot(1) > 0 
    v(1) = stat_k_roh * stat_roh;
else
    v(1) = - stat_k_roh * stat_roh;
end 
v(2) = 0; 
v(3) = stat_k_alpha * stat_alpha + stat_k_phi * stat_phi; 

%% TEST :: Lock direction once aimed in the right way
[a,b] = size(betaSetpoint); 
if a == 1 && b == 1  
    if interimTargetLength < pushLength % while positioning
        if sqrt( (carrot(2)*cos(carrot(3)-pi/2)/sin(carrot(3)-pi/2))^2 + carrot(2)^2 ) < 0.02 && ... % ytolerance in [m]
                abs(betaSetpoint) < 0.01 && ... % in [rad]
                abs(carrot(3)) < 0.0 || locked
            v(2) = 0;
            v(3) = 0;
            locked = true; 
        end
    end
end

%% scale the vector
if finalTargetLength > 0 && interimTargetLength > 0 
    v = v.*(finalTargetLength/interimTargetLength); % scale up to very high numbers
end
tempSpeed = sqrt( v(1)^2 + v(2)^2 );
if tempSpeed > targetSpeed
    v = v.*(targetSpeed/tempSpeed); % scale down to target speed
end
