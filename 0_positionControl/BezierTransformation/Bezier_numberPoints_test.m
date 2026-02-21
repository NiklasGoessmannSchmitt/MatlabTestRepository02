function  Bezier_numberPoints_test 
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% arc= [2617.42709084947,1561.24308034589,1.85000000000000,0,90];
%arc= [2617.42709084947,1561.24308034589,1.85000000000000,225,315];
%arc= [0, 0,1,0,45];
arc= [0,0,1,90,45];
%arc= [0,0,1,270,0];
arc_help=[0,0,arc(3), arc(4), arc(5)];
% arc=[0,0,1, 0, 45];
%% Start- und Endpunkte berechnen
x = arc_help(1)/1000;
y = arc_help(2)/1000;
radius = arc_help(3);

if (arc_help(4) < arc_help(5))
    a_Start_deg = arc_help(4);                           %Startwinkel
    a_End_deg = arc_help(5);                             %Endwinkel
else
    a_Start_deg = arc_help(5);                           %Startwinkel
    a_End_deg = arc_help(4);                             %Endwinkel
end

if abs( arc_help(4) - arc_help(5)) > 170 %(arc_help(4) => 270) && (arc_help(5) <= 10 )) || (arc_help(5) => 270) && (arc_help(4) <= 10 )
    if arc_help(4) >= 270
        a_Start_deg= arc_help(4);
        a_End_deg= arc_help(5);
    else 
        a_Start_deg= arc_help(5);
        a_End_deg= arc_help(4);
        
    end
end 



a_Start=deg2rad(a_Start_deg);                   %Umrechnung in Rad
a_End=deg2rad(a_End_deg);


    X1=x+cos(a_Start)*radius;                  %Anfangspunkt berechnen
    Y1=y+sin(a_Start)*radius;

    X4=x+cos(a_End)*radius;                    %Endpunkt berechnen
    Y4=y+sin(a_End)*radius;



%% Mittlere Punkte

if abs(a_End_deg -a_Start_deg)>=180
    if a_End_deg==0
        angle= abs(360 - a_Start_deg);
    else
        angle= abs(a_End_deg - a_Start_deg)-180;
    end
else
    angle= abs(a_End_deg -a_Start_deg);
end


% Calculate sin and cosine for half that angle
    sin_1 = sin(pi * angle / 180 / 2);
    cos_1 = cos(pi * angle / 180 / 2);
%Find the points and weight
%     P1(1) = X1;
%     P1(2) = Y1;
%     P2(1) = 0; %x+cos((a_Start-a_End)/2)*radius;
%     P2(2) = - radius/cos_1; % y+sin((a_Start-a_End)/2)*radius;
%     P3(1) = X4;
%     P3(2) = Y4;


    P1(1) = (-radius * sin_1 );
    P1(2) = -radius * cos_1 ;
    P2(1) = 0;
    P2(2) = - radius / cos_1;
    P3(1) = radius * sin_1; 
    P3(2) = -radius * cos_1;
    weight = (cos_1);
    P.x=[P1(1),P2(1),P3(1)];
    P.y=[P1(2),P2(2),P3(2)];
    n=100;
    granularity = 1/n;
    x = zeros(1,n);
    y = zeros(1,n);
    b = 1;



%     %Test weight 
%     P.x(1)=200;
%     P.x(3)=5;
%     P.x(2)=(P.x(1)+P.x(3))/2;
%     
% 
%     P.y(1)=300;
%     P.y(3)=7;
%     P.y(2)=(P.x(1)+P.x(3))/2;
%     
%     weight=0;
Punkt.x=P.x;
Punkt.y=P.y;

    mitte= a_End_deg-(angle/2);


    angle_rot=deg2rad(mitte-270);


P.x=(Punkt.x*cos(angle_rot) - Punkt.y*sin(angle_rot));
P.y=(Punkt.x*sin(angle_rot) + Punkt.y*cos(angle_rot));

 
%weight=0;

    for t = 0:granularity:1
        d=  (1-t)^2 + 2*weight*t*(1- t) + t^2;
        x(b) = ((1-t)^2*P.x(1) + 2*t*weight*(1-t)*P.x(2) + t^2*P.x(3))/d;
        y(b) = ((1-t)^2*P.y(1) + 2*t*weight*(1-t)*P.y(2) + t^2*P.y(3))/d;
        b = b + 1;
    end

   % plot(x,y, 'r');
   % hold on;
   GraphsDeviation(x,y, P)
   [l,m]= bezier3(P.x, P.y, 100); 
   figure;
   axis equal;
   hold on;
   plot(l,m,'r', 'DisplayName', 'ohne Gewichtung', LineWidth=3);          %Kreissegment plotten
   plot(x,y, 'b--', 'DisplayName', 'mit Gewichtung', LineWidth=3);               %Bezier Kurve plotten
   xlabel('x');
   ylabel('y');
   legend;
    %plot(x,y, 'b');
    
%% Punkte zurück zu Mittelpunkt schieben
% P.x=P.x+ (arc(1));
% P.y=P.y+ (arc(2));

% plot(P.x, P.y , 'mo', 'DisplayName', 'Stützpunkte',LineWidth=2);
% hold on
% grid on
% axis equal

%% Plot aus DXF-Tool als Referenz
% x_dxf = arc(1);
% y_dxf = arc(2);
% r = arc(3);
% a1 = arc(4);
% a2 = arc(5);
% 
% if a2<a1
%     a2 = a2 + 360;
% end
% 
% theta = deg2rad(linspace(a1,a2,100));
% X_dxf = x_dxf + r*cos(theta);
% %Xabs = abs(X_dxf);
% Y_dxf = y_dxf + r*sin(theta);
% %Yabs = abs(Y);
% 
% 
% % plot(X_dxf,Y_dxf,'r', 'DisplayName', 'Kreissegment', LineWidth=3);
%% Drei Bezier Punkte simulieren
% P=[];
% 
% X1=X1+arc(1);
% X4=X4+arc(1);
% Y1=Y1+arc(2);
% Y4=Y4+arc(2);
% P.x=[X1,X1,X4];
% P.y=[Y1,Y4,Y4];
%% Bezier Punkte und Plot
%[xq, yq, dq, endIDindx, xAGV, yAGV, aAGV, b] = getTrackExternal(P);
%[x,y] = bezier3(P.x,P.y,100);
% plot(x,y, 'b--', 'DisplayName', 'Bezier Kurve', LineWidth=3);
% plot(arc(1), arc(2), 'r+','DisplayName', 'Mittelpunkt', LineWidth=2); %Mittelpunkt kreissegment

%% Plot ertellen
% plot(P.x, P.y , 'mo', 'DisplayName', 'Stützpunkte',LineWidth=2);            %Stützpunkte plotten
% hold on
% grid on
% axis equal
% plot(X_dxf,Y_dxf,'r', 'DisplayName', 'Kreissegment', LineWidth=3);          %Kreissegment plotten
% plot(x,y, 'b--', 'DisplayName', 'Bezier Kurve', LineWidth=3);               %Bezier Kurve plotten
% plot(arc(1), arc(2), 'r+','DisplayName', 'Mittelpunkt', LineWidth=2);       %Mittelpunkt Kreissegment
% xlabel('x');
% ylabel('y');
% legend;



%% Abweichung berechnen;
%GraphsDeviation(X_dxf,Y_dxf, P);

end