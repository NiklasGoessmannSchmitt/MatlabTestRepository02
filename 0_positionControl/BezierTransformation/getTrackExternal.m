function [x, y, w, endIDindex, xAGV, yAGV, aAGV, b] = getTrackExternal(BezierPunkte) 

endIDindex = 0;

xAGV = 0;
yAGV = 0;
aAGV = 0;

b=BezierPunkte;




x=[];
y=[];

n = 100;
for i = 1:length(BezierPunkte) 
    [xq ,yq] = bezier3(BezierPunkte{1,i}.x, BezierPunkte{1,i}.y,n); 
   
%     xq = xq/1000; 
%     yq = yq/1000;

    x = [x, xq(1:end-1)]; 
    y = [y, yq(1:end-1)]; 
end 
endIDindex = length(y)-100;

% 
% if strcmp(name, 'basic')
%     P.x=[0 0 2];
%     P.y=[0 2 2];
%     n=100;
%     [xq1, yq1] = bezier3(P.x,P.y,n);
% 
%     P1.x=[2 3 4];
%     P1.y=[2 2 2];
%     [xq2, yq2] = bezier3(P1.x,P1.y,n);
% 
%     P2.x=[4 6 6];
%     P2.y=[2 2 0];
%     [xq3, yq3] = bezier3(P2.x,P2.y,n);
% 
%     P3.x=[6  6  4];
%     P3.y=[0 -2 -2];
%     [xq4, yq4] = bezier3(P3.x,P3.y,n);
% 
%     P4.x=[4 3 2];
%     P4.y=[-2 -2 -2];
%     [xq5, yq5] = bezier3(P4.x,P4.y,n);
%     
%     P5.x=[ 2  0 0];
%     P5.y=[-2 -2 0];
%     [xq6, yq6] = bezier3(P5.x,P5.y,n);
% 
%     x = [xq1(1:end-1), xq2(1:end-1), xq3(1:end-1), xq4(1:end-1), xq5(1:end-1), xq6(1:end)];
%     y = [yq1(1:end-1), yq2(1:end-1), yq3(1:end-1), yq4(1:end-1), yq5(1:end-1), yq6(1:end)];
%     b= {P, P1, P2, P3, P4, P5};
%     xAGV = 0.0;
%     yAGV = 0.5;
%     aAGV = pi/3;
%     endIDindex = length(y)-100;% [0, 4];
%     
% elseif strcmp(name, 'segment')
%     Px=[0 0 2];
%     Py=[0 2 2];
%     n=1000;
%     [xq1, yq1] = bezier(Px,Py,n);
% 
%     xq2 = linspace(2,5,n);
%     yq2 = 2*ones(1,length(xq2));
% 
%     x = [xq1(1:end-1), xq2(1:end-1)];
%     y = [yq1(1:end-1), yq2(1:end-1)];
%     
% elseif strcmp(name, 'straight')
%     xAGV = 0; 
%     yAGV = 0.2; 
%     aAGV = 0; 
%     
%     n = 1000;
%     x = linspace(0,5,n);
%     y = zeros(1,length(x));
%     
%     endIDindex = [1, 1];
%     
% elseif strcmp(name, 'BMF')
%     xAGV = 0; 
%     yAGV = 0; 
%     aAGV = 0; 
%     
%     n = 2000;
%     xq1 = linspace(-2,2,n);
%     yq1 = zeros(1,length(xq1));
%     
%     n = 1;
%     for i = -90:0.1:90
%         xq2(n) = 2.0 + 1.5 * cos(i/180*pi); 
%         yq2(n) = 1.5 + 1.5 * sin(i/180*pi); 
%         n = n + 1;
%     end
%     
%     xq3 = linspace(2,-2,n);
%     yq3 = 3*ones(1,length(xq3));
% 
%     x = [xq1(1:end-1), xq2(1:end-1), xq3(1:end)];
%     y = [yq1(1:end-1), yq2(1:end-1), yq3(1:end)];
% 
%     endIDindex = length(y)-100;% [0, 4];
% 
% end

% Abstände zw den Punkten w
w = zeros(1,length(x)-1);

for i= 1:length(x)-1
    w(i) = sqrt ( (x(i+1) - x(i))^2 + (y(i+1) - y(i))^2 );
end

xAGV = x(100);
yAGV = y(100);
aAGV = pi/6;

end
