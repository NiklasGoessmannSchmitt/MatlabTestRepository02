function [layer] = selectLayer(dxf, string)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% outputArg1 = inputArg1;
% outputArg2 = inputArg2;

%layer Anlegen
layer={};

ne=dxf.ne;


%Mögliche Formelemente festlegen
Element2=["POINT","LINE","CIRCLE","ARC", "ELLIPSE","SPLINE"];       %"LWPOLYLINE", wurde herausgenommen

k=1;

for i = 1:ne                                                        %Alle übergebenen Werte durchlaufen
    %str=string;
    if any(dxf.entities(i).layer==string) && any(dxf.entities(i).name==Element2)    %Überprüfen auf ausgewählte Layer und ob es sich um ein zulässiges Formelement handelt
        layer.entities(k)= dxf.entities(i);                         %Werte in Array schreiben
        k=k+1;
    end
end


end