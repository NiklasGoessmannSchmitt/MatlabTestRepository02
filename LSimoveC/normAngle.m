function y = normAngle(x)

% Normiere Winkel -..+ -> -PI .. +PI
% 
% 
%  Automatically Edited - Old code marked with //~ - (UTC) 20.06.2018 14:59:48
% SVN-Keywords
%     $Author: merten $
%     $Date: 2011-06-20 23:35:24 +0200 (Mo, 20 Jun 2011) $
%     $Revision: 4325 $    
%     $Id: C_UtilNormAngle.SCL 4325 2011-06-20 21:35:24Z merten $
%     $HeadURL: svn+ssh://sunshine.mchp.siemens.de/project/db2/projects/afl/tags/20111203_Daimler_Sindelfingen/src_plc/common/C_UtilNormAngle.SCL $
% SVN-Keywords~ CONST
%     TWO_PI        := DINT#628319;
%     PI            := DINT#314159;
%     PI_HALF       := DINT#157079;
%     THREE_HALF_PI := DINT#471239;
% END_CONST
% *)
if (x < -pi) 
    y = mod((x + pi), 2*pi) + pi;
elseif (x >= pi) 
   y = mod((x + pi), 2*pi) - pi;
else
    y = x; 
end

% (*Beispiel Winkel von 45° (PI_VIERTEL)
% -> Keine IF-Schleife gebe 45° zurück
% Beispiel Winkel von 270° (3 PI_HALF)
% -> 2. IF-Schleife wird durchlaufen 
% -> Util_NormAngle := (3*PI/2 MOD 2PI) - PI = PI/2 - PI = -90°
% 
% --> Alle Winkel liegen zwischen [-180; +180]also zwischen [-PI ; PI] 