function [vRamped, accBit, decBit] = fbRampe (vTarget, vOld, acc, dec, SamplingTime)
% [vRamped, accBit, decBit] = fbRampe (vTarget, vOld, acc, dec, SamplingTime)
%
% (*******************************************************************
% // Beschreibung
% //   Erzeugt eine Rampe
% //
% // Inhalt
% //
% //
% // Version
% // V1.0            JB: Ersterstellung
% // V1.1 2013-12-04 JB: Erweiterung um RESET-Eingang, um den
% //                     Ausgangswert sofort dem Eingang nachzuführen
% // V2.0 2018-04-06 JB: auf TIA-Portal / SCL angepasst
% // V2.1 2018-11-12 DG: angepasst auf Verwendung in XYA-Rampe
% // V2.1 2020-04-15 DG: adjustments on negative acceleration and
% //                                    positive deceleration 
% //
% // Autoren
% // JB: Johannes Brandt
% // DG: Daniel Gauglitz
% //
% // © Alle Rechte vorbehalten/All rights reserved, Siemens AG, 2018
% *******************************************************************)

tempBeschleunigung = false;
tempVerzoegerung   = false;

addent     = SamplingTime * acc;
subtrahent = SamplingTime * dec;

% addieren oder subtrahieren
if vTarget > vOld
    
    if vOld >= 0 
        vRamped = vOld + addent;
        tempBeschleunigung = true;
        if vRamped > vTarget % nach Addition über #SOLL
            vRamped = vTarget;
        end
    else
        vRamped = vOld + subtrahent;
        tempVerzoegerung = true;
        if vRamped > vTarget
            vRamped = vTarget; 
        end
    end
        
elseif vTarget < vOld 
    
    if vOld >= 0 
        vRamped = vOld - subtrahent;
        tempVerzoegerung = true;
        if vRamped < vTarget % nach Subtraktion unter #SOLL
            vRamped = vTarget;
        end
    else
        vRamped = vOld - addent; 
        tempBeschleunigung = true; 
        if vRamped < vTarget
            vRamped = vTarget; 
        end
    end
else
    vRamped = vOld;
end

% Bits für Verzögerung und Beschleunigung schreiben
decBit = tempVerzoegerung;
accBit = tempBeschleunigung;
