clc; close all;
clear all;


%____________Plotten des Layoutes mit DXFtool_______________
% read file and plot

%Layout = DXFtool('Testcenter_AGV_Anlage.dxf');
%Layout = DXFtool('Layout_2.dxf');
Layout= DXFtool('Fahrkurse_Linienmodell_neu.dxf');

%________Vorzeichen korrigieren 
% for i=1:Layout.ne                           %Durch alle Einträge
%     str=convertCharsToStrings ('ARC');
%     if Layout.entities(i).name == str
%         Layout.entities(i).arc(1) =abs(Layout.entities(i).arc(1));      %X Wert ablosut setzten
%     end
% end




%m =transformbezier(dxf);
AuswahlLayer=possibleLayers(Layout);

%Layer=selectLayer(Layout, "Fahrkurs_1_als_Linie");
%string= ["Fahrkurs_1_als_Linie"];
string= ["F8_123001"];
Layer=selectLayer(Layout, string);
m= length(Layer.entities);

BezierPunkte=transform(Layer, m);
% for i=1:m
%     plot(BezierPunkte{1,i}.x, BezierPunkte{1,i}.y, 'ro')
% hold on;
% end

Sortierte_Punkte=Bezier_Reihenfolge(BezierPunkte,m);



 
[xq, yq, dq, endIDindx, xAGV, yAGV, aAGV, b] = getTrackExternal(Sortierte_Punkte);
plot((xq*1000),(yq*1000), 'b');
axis equal;
hold on;



for i=1:length(Sortierte_Punkte)
    x=Sortierte_Punkte{1,i}.x * 1000;
    y=Sortierte_Punkte{1,i}.y * 1000;
    plot(x,y,'ro');

end


