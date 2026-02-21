function [c] = transform(dxf, ne)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

   
for i = 1:ne



    switch dxf.entities(i).name
        case 'POINT'
            bezier = transform_point(dxf.entities(i).point);
        case 'LINE'
            bezier = transform_line(dxf.entities(i).line);
        case 'LWPOLYLINE'
            bezier = transform_poly(dxf.entities(i).poly);
        case 'CIRCLE'
            bezier = transform_circle(dxf.entities(i).circlex);
        case 'ARC'
            bezier = transform_arc(dxf.entities(i).arc);
%         case 'ELLIPSE'
%             bezier = transform_ellipse(dxf.entities(i).ellipse);
%         case 'SPLINE'
%             bezier = transform_spline(dxf.entities(i).spline);
    end

    if i == 1
        c={bezier};
    else
        c=[c,{bezier}];
    end

end

%% Point-------------------------------------------------------------------
    function P= transform_point(point)
        X1= point(1);
        Y1= point(2);
        
         P.x=[X1];
         P.y=[Y1];
                 
        
    
    end

%% Line--------------------------------------------------------------------
    function P= transform_line(line)
        %Line(Xi,Yi,Xj,Yj)
        Xi=line(1);
        Yi=line(2);
        Xj=line(3);
        Yj=line(4);

        %Rechnen

        X1=Xi;
        Y1=Yi;

        X3=Xj;
        Y3=Yj;

        X2=(X1+X3)/2;
        Y2=(Y1+Y3)/2;

        %Speichern der X und Y Werte in Cell Array
        P.x=[X1,X2,X3]/1000; % [mm] --> [m]
        P.y=[Y1,Y2,Y3]/1000; % [mm] --> [m]


        P.weight=0; 

        



    end

%% LWPOLYLINE--------------------------------------------------------------
    function  P= transform_poly(poly)
        %vertices X,Y array [n_verts,2]
        %Ändern?!


        X1=poly(1);
        Y1=poly(2);

        X3=poly(3);
        Y3=poly(4);

        X2=(X1+X3)/2;
        Y2=(Y1+Y3)/2;



        %Speichern der X und Y Werte in Cell Array
        P.x=[X1,X2,X3]/1000; % [mm] --> [m];
        P.y=[Y1,Y2,Y3]/1000; % [mm] --> [m];
        
        
    end
    
%% CYCLE-------------------------------------------------------------------
    function P= transform_circle(circle)
        % plot circles: (X Center,Y Center,Radius)
        x = circle(1);
        y = circle(2);
        r = circle(3);

        X1= x+r;
        Y1= y+r;
        X3= x-r;
        Y3= y-r;
        %X2=x+r;
        %Y2=y-r;
        
        P.X=[X1,X2,X3]/1000; % [mm] --> [m];
        P.Y=[Y1,Y2,Y3]/1000; % [mm] --> [m];
        
      


    end

%% ARC---------------------------------------------------------------------
    function P= transform_arc (arc)
        %ARC (Xc, Yc, R, begin angle, end angle)

        %% Version mit 3 Punkten
        arc_help=[0,0,arc(3), arc(4), arc(5)];  %in mm

        %% Start- und Endpunkte berechnen
        x = arc_help(1)/1000;                           %mm in m
        y = arc_help(2)/1000;
        radius = arc_help(3)/1000;

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


        if abs(a_End_deg -a_Start_deg)>=180
            if a_End_deg==0
                angle= abs(360 - a_Start_deg);
            else
                angle= abs(a_End_deg - a_Start_deg)-180;
            end
        else
            angle= abs(a_End_deg -a_Start_deg);
        end
        %% Calculate sin and cosine for half that angle
        sin_1 = sin(pi * angle / 180 / 2);
        cos_1 = cos(pi * angle / 180 / 2);
        %%Find the points and weight
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
%         n=100;
%         granularity = 1/n;
%         x = zeros(1,n);
%         y = zeros(1,n);
%         b = 1;

%% Punkt zurück rotieren
            mitte= a_End_deg-(angle/2);
            
            if abs(a_End_deg-a_Start_deg)< 180
                angle_rot=(deg2rad((mitte+ 270)));
            else
                angle_rot=(deg2rad((mitte+270)));
            end
            
            if a_Start_deg<180
                angle_rot=deg2rad(mitte -270); %270-mitte);
            else
                angle_rot=deg2rad(mitte-270);
            end
            
            Punkt.x=P.x;
            Punkt.y=P.y;
            P.x=(Punkt.x*cos(angle_rot) - Punkt.y*sin(angle_rot));
            P.y=(Punkt.x*sin(angle_rot) + Punkt.y*cos(angle_rot));

%% Punkte zurück zu Mittelpunkt schieben

        P.x=P.x+ (arc(1)/1000);
        P.y=P.y+ (arc(2)/1000);

        P.weight=weight;


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Neue Version mit 4 Punkten
%         arc_help=[0,0,arc(3), arc(4), arc(5)];  %in mm
% 
%         %% Start- und Endpunkte berechnen
%         x = arc_help(1)/1000;                           %mm in m
%         y = arc_help(2)/1000;
%         radius = arc_help(3)/1000;
%         %         a_Start_deg = arc_help(4);                           %Startwinkel
%         %         a_End_deg = arc_help(5);                             %Endwinkel
%         
%         if (arc_help(4) < arc_help(5))
%             a_Start_deg = arc_help(4);                           %Startwinkel
%             a_End_deg = arc_help(5);                             %Endwinkel
%         else
%             a_Start_deg = arc_help(5);                           %Startwinkel
%             a_End_deg = arc_help(4);                             %Endwinkel
%         end
% 
%         if abs( arc_help(4) - arc_help(5)) > 170 %(arc_help(4) => 270) && (arc_help(5) <= 10 )) || (arc_help(5) => 270) && (arc_help(4) <= 10 )
%             if arc_help(4) >= 270
%                 a_Start_deg= arc_help(4);
%                 a_End_deg= arc_help(5);
%             else
%                 a_Start_deg= arc_help(5);
%                 a_End_deg= arc_help(4);
% 
%             end
%         end
% 
%         a_Start=deg2rad(a_Start_deg);                   %Umrechnung in Rad
%         a_End=deg2rad(a_End_deg);
% 
%         %         if a_Start_deg < a_End_deg
%         %             X1=x+cos(a_Start)*radius;                  %Anfangspunkt berechnen
%         %             Y1=y+sin(a_Start)*radius;
%         %
%         %             X4=x+cos(a_End)*radius;                    %Endpunkt berechnen
%         %             Y4=y+sin(a_End)*radius;
%         %         elseif a_Start_deg > a_End_deg
%         %             X4=x+cos(a_Start)*radius;                  %Endpunkt berechnen
%         %             Y4=y+sin(a_Start)*radius;
%         %
%         %             X1=x+cos(a_End)*radius;                    %Anfangspunkt berechnen
%         %             Y1=y+sin(a_End)*radius;
%         %
%         %         else
%         %             disp('no valide circle');
%         %         end
%         %if  (a_Start_deg>=270  && a_End_deg < 90)
% 
%             X1=x+cos(a_Start)*radius;                  %Anfangspunkt berechnen
%             Y1=y+sin(a_Start)*radius;
% 
%             X4=x+cos(a_End)*radius;                    %Endpunkt berechnen
%             Y4=y+sin(a_End)*radius;
% 
% 
% 
%         % Mittlere Punkte
% 
%         %angle= abs(a_End_deg -a_Start_deg);                  %Oeffnungswinkel
%         if abs(a_End_deg -a_Start_deg) <= 180 
%             angle= abs(a_End_deg -a_Start_deg);                  %Oeffnungswinkel
%            
%         else %if (a_End_deg >= 270 && a_Start_deg >= 0) || (a_Start_deg >= 270 && a_End_deg>=0)
%             angle =360-(abs(a_End_deg -a_Start_deg) );
%         end
% 
% 
%         length_L= radius*4*tan((pi*angle /180/4))/3;
% 
%         P1= [X1 Y1];
%         P4= [X4 Y4];
% 
%         P1_magnitude = sqrt(P1(1)^2 + P1(2)^2);
%         P1_norm_test = P1/P1_magnitude;
% 
%         P4_magnitude = sqrt(P4(1)^2 + P4(2)^2);
%         P4_norm_test = P4/P4_magnitude;
% 
%         P2(1)= P1(1)+(-length_L*P1_norm_test(2));
%         P2(2)= P1(2)+(+length_L*P1_norm_test(1));
% 
%         P3(1)= P4(1)+(length_L*P4_norm_test(2));
%         P3(2)= P4(2)+(-length_L*P4_norm_test(1));
% 
% 
%         P.x=[P1(1),P2(1),P3(1), P4(1)];
%         P.y=[P1(2),P2(2),P3(2), P4(2)];
% 
%         %% Punkte zurück zu Mittelpunkt schieben
%         P.x=P.x+ (arc(1)/1000);
%         P.y=P.y+ (arc(2)/1000);
% 
% %% test
%         x_dxf = arc(1);
% y_dxf = arc(2);
% r = arc(3);
% a1 = arc(4);
% a2 = arc(5);
% if a2<a1
%     a2 = a2 + 360;
% end
% 
% theta = deg2rad(linspace(a1,a2,100));
% X_dxf = x_dxf + r*cos(theta);
% %Xabs = abs(X_dxf);
% Y_dxf = y_dxf + r*sin(theta);



       


    end

     %% ELLIPSE------------------------------------------------------------
%     function k= transform_ellipse(ellipse)
%     %Ellipse(Xc, Yc, Xe, Ye, ratio, begin angle, end angle)
%            X1=ellipse(1);
%            Y1=ellipse(2);
% 
%            X3=ellipse(3);
%            Y3=ellipse(4);
% 
%            X2=
%            Y2=
%
%         P.X=[X1,X2,X3]/1000; % [mm] --> [m];
%         P.Y=[Y1,Y2,Y3]/1000; % [mm] --> [m];
% 
%        
% 
%      end
% 
%     %SPLINE
%     function k= transform_spline(spline)
% 
%         P.X=[X1,X2,X3]/1000; % [mm] --> [m];
%         P.Y=[Y1,Y2,Y3]/1000; % [mm] --> [m];
%
%      end
%  


end