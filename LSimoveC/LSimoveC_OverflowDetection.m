function ticks = LSimoveC_OverflowDetection (enc, encPrev, encMax, encMin) 
% ticks = LSimoveC_OverflowDetection (enc, encPrev, encMax, encMin) 
%
% //=============================================================================
% //Siemens AG
% //(c)Copyright 2018 All rights reserved
% //-----------------------------------------------------------------------------
% //Library: LSimoveC
% //Function:
% //      This function block detects encoder overflows and gives back the 
% //      appropriate number of encoder ticks in all cases. 
% //      
% //      The maximum value range is DINT to respect the value range of an S7-1200
% //-----------------------------------------------------------------------------
% //Change journal:
% //V3.0 2020-05-20 CP Initial Draft
% //
% //=============================================================================

if abs(enc - encPrev) < ((encMax + abs(encMin)) / 3) % normal Case
    ticks = enc - encPrev;
elseif encPrev < enc % underflow
    ticks = encMin - encPrev - encMax + enc;
elseif encPrev > enc % overflow
    ticks = -encMin + enc + encMax - encPrev;
else
    ticks = 0; 
end