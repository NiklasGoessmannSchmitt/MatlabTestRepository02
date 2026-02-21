function [x, y, w, endIDindex, xAGV, yAGV, aAGV, b, bwd] = getTrack(name,radius) 
% [x, y] = getTrack(name)
%
% known AGV types ('name'): 
% - basic
% - segment
% - straight

endIDindex = 0; 

xAGV = 0; 
yAGV = 0; 
aAGV = 0;

bwd = 0; 

if strcmp(name, 'basic')
    P.x=[0 0 2];
    P.y=[0 2 2];
    n=100;
    [xq1, yq1] = bezier3(P.x,P.y,n);

    P1.x=[2 3 4];
    P1.y=[2 2 2];
    [xq2, yq2] = bezier3(P1.x,P1.y,n);

    P2.x=[4 6 6];
    P2.y=[2 2 0];
    [xq3, yq3] = bezier3(P2.x,P2.y,n);

    P3.x=[6  6  4];
    P3.y=[0 -2 -2];
    [xq4, yq4] = bezier3(P3.x,P3.y,n);

    P4.x=[4 3 2];
    P4.y=[-2 -2 -2];
    [xq5, yq5] = bezier3(P4.x,P4.y,n);
    
    P5.x=[ 2  0 0];
    P5.y=[-2 -2 0];
    [xq6, yq6] = bezier3(P5.x,P5.y,n);

    x = [xq1(1:end-1), xq2(1:end-1), xq3(1:end-1), xq4(1:end-1), xq5(1:end-1), xq6(1:end)];
    y = [yq1(1:end-1), yq2(1:end-1), yq3(1:end-1), yq4(1:end-1), yq5(1:end-1), yq6(1:end)];
    b= {P, P1, P2, P3, P4, P5};
    xAGV = 0.0;
    yAGV = 0.5;
    aAGV = pi/2;
    endIDindex = length(y)-50;% [0, 4];
    
elseif strcmp(name, 'segment')
    Px=[0 0 2];
    Py=[0 2 2];
    n=1000;
    [xq1, yq1] = bezier(Px,Py,n);

    xq2 = linspace(2,5,n);
    yq2 = 2*ones(1,length(xq2));

    x = [xq1(1:end-1), xq2(1:end-1)];
    y = [yq1(1:end-1), yq2(1:end-1)];
    
elseif strcmp(name, 'sCurve')
    xAGV = 0.5; 
    yAGV = 0; 
    aAGV = 0; 

    straight = 2; 
%     radius = 2; 
    n = 1000;

    a.x = 0; 
    a.y = 0; 
    b.x = straight/2; 
    b.y = 0; 
    c.x = straight; 
    c.y = 0; 
    P.x=[a.x b.x c.x];
    P.y=[a.y b.y c.y];
    [xq0, yq0] = bezier3(P.x,P.y,n);

    a = c; 
    b.x = a.x + radius; 
    b.y = a.y; 
    c.x = b.x; 
    c.y = b.y + radius;    
    P1.x=[a.x b.x c.x];
    P1.y=[a.y b.y c.y];
    [xq1, yq1] = bezier3(P1.x,P1.y,n);

    a = c; 
    b.x = a.x; 
    b.y = a.y + radius; 
    c.x = b.x + radius; 
    c.y = b.y; 
    P2.x=[a.x b.x c.x];
    P2.y=[a.y b.y c.y];
    [xq2, yq2] = bezier3(P2.x,P2.y,n);

    a = c; 
    b.x = a.x + straight; 
    b.y = a.y; 
    c.x = b.x + straight; 
    c.y = b.y; 
    P3.x=[a.x b.x c.x];
    P3.y=[a.y b.y c.y];
    [xq3, yq3] = bezier3(P3.x,P3.y,n);

    x = [xq0(1:end-1), xq1(1:end-1), xq2(1:end-1), xq3(1:end)];%, xq4(1:end)];
    y = [yq0(1:end-1), yq1(1:end-1), yq2(1:end-1), yq3(1:end)];%, yq4(1:end)];
    b= {P, P1, P2, P3};%, P4};
    endIDindex = length(y)-20;% [0, 4];

elseif strcmp(name, 'cCurve')
    xAGV = -5; 
    yAGV = 0; 
    aAGV = 0; 
    
    n = 1000;
%    xq4 = linspace(-2,0,n);
%    yq4 = zeros(1,length(xq4));
 
    P.x=[-5 -1 0];
    P.y=[0 0 0];
    [xq4,yq4] = bezier3(P.x,P.y,n);

    P1.x=[0 radius radius];
    P1.y=[0   0    radius];
    [xq1, yq1] = bezier3(P1.x,P1.y,n);

%    xq2 = radius.*ones(1,n); 
%    yq2 = linspace(radius,radius+3,n);
 
    P2.x=[radius radius   radius];
    P2.y=[radius radius+1 radius+15];
    [xq2,yq2] = bezier3(P2.x,P2.y,n);

    x = [xq4(1:end-1), xq1(1:end-1), xq2(1:end)];
    y = [yq4(1:end-1), yq1(1:end-1), yq2(1:end)];
    b= {P, P1, P2};
    endIDindex = length(y)-100;% [0, 4];

elseif strcmp(name, 'pCurve')
    xAGV = -2; 
    yAGV = 0; 
    aAGV = 0; 
    
    n = 1000;
 
    P.x=[-2 -1 0];
    P.y=[0 0 0];
    [xq0,yq0] = bezier3(P.x,P.y,n);

    P1.x=[0 radius radius];
    P1.y=[0   0    radius];
    [xq1, yq1] = bezier3(P1.x,P1.y,n);
 
    P2.x=[radius radius          radius-radius/4];
    P2.y=[radius radius+radius/6 radius+radius/2];
    [xq2,yq2] = bezier3(P2.x,P2.y,n);

    P3.x=[radius-radius/4 radius-radius/2     radius-radius/2];
    P3.y=[radius+radius/2 radius+radius*0.8   radius*2.5];
    [xq3,yq3] = bezier3(P3.x,P3.y,n);

    P4.x=[radius-radius/2 radius-radius/2   radius-radius/2];
    P4.y=[radius*2.5 radius*4 radius*5];
    [xq4,yq4] = bezier3(P4.x,P4.y,n);

    x = [xq0(1:end-1), xq1(1:end-1), xq2(1:end-1), xq3(1:end-1), xq4(1:end)];
    y = [yq0(1:end-1), yq1(1:end-1), yq2(1:end-1), yq3(1:end-1), yq4(1:end)];
    b= {P, P1, P2, P3, P4};
    endIDindex = length(y)-100;% [0, 4];
    

elseif strcmp(name, 'straight')
    xAGV = 1; 
    yAGV = 0.05; 
    aAGV = 0; 
    
    n = 1000;
 
    P.x=[0 0.5 1]*10;
    P.y=[0 0  0];
    [xq0,yq0] = bezier3(P.x,P.y,n);

    x = [xq0(1:end-1)];
    y = [yq0(1:end-1)];
    b= {P};
    endIDindex = length(y)-100;
    
elseif strcmp(name, 'BMF')
    xAGV = 0; 
    yAGV = 0; 
    aAGV = 0; 
    
    n = 2000;
    xq1 = linspace(-2,2,n);
    yq1 = zeros(1,length(xq1));
    
    n = 1;
    for i = -90:0.1:90
        xq2(n) = 2.0 + 1.5 * cos(i/180*pi); 
        yq2(n) = 1.5 + 1.5 * sin(i/180*pi); 
        n = n + 1;
    end
    
    xq3 = linspace(2,-2,n);
    yq3 = 3*ones(1,length(xq3));

    x = [xq1(1:end-1), xq2(1:end-1), xq3(1:end)];
    y = [yq1(1:end-1), yq2(1:end-1), yq3(1:end)];

    endIDindex = length(y)-100;% [0, 4];

elseif strcmp(name, 'EWN_1')

    P.x=[3.42 10 22];
    P.y=[7 7 7];
    n=1000;
    [xq1, yq1] = bezier3(P.x,P.y,n);

    P1.x=[22 23.8 23.8];
    P1.y=[7 7 9];
    [xq2, yq2] = bezier3(P1.x,P1.y,n);

    P2.x=[23.8 23.8 23.8];
    P2.y=[9 15 18.3];
    [xq3, yq3] = bezier3(P2.x,P2.y,n);

    x = [xq1(1:end-1), xq2(1:end-1), xq3(1:end-1)];
    y = [yq1(1:end-1), yq2(1:end-1), yq3(1:end-1)];
    b= {P, P1, P2};%, P3, P4, P5};
    xAGV = 3.42;
    yAGV = 7;
    aAGV = 0;
    endIDindex = length(y);

elseif strcmp(name, 'EWN_2')

    P.x=[23.8 22 21];
    P.y=[18.3 18.3 18.3];
    n=1000;
    [xq1, yq1] = bezier3(P.x,P.y,n);

    x = [xq1(1:end-1)];
    y = [yq1(1:end-1)];
    b= {P};
    xAGV = 23.8;
    yAGV = 18.7;
    aAGV = 0;
    endIDindex = length(y);

    bwd = 1; 

elseif strcmp(name, 'EWN_3')

    P.x=[21 22 23.8];
    P.y=[18.3067 18.3067 18.3067];
    n=1000;
    [xq1, yq1] = bezier3(P.x,P.y,n);

    x = [xq1(1:end-1)];
    y = [yq1(1:end-1)];
    b= {P};
    xAGV = 21;
    yAGV = 18.7;
    aAGV = 0;
    endIDindex = length(y);

elseif strcmp(name, 'EWN_4')

    P.x=[23.8 23.8 23.8];
    P.y=[18.3067 10 2.1];
    n=1000;
    [xq1, yq1] = bezier3(P.x,P.y,n);

    x = [xq1(1:end-1)];
    y = [yq1(1:end-1)];
    b= {P};
    xAGV = 21;
    yAGV = 18.7;
    aAGV = 0;
    endIDindex = length(y);

elseif strcmp(name, 'EWN_5')

    P.x=[23.8 24 27.5];
    P.y=[2.1 2.1 2.1];
    n=1000;
    [xq1, yq1] = bezier3(P.x,P.y,n);

    x = [xq1(1:end-1)];
    y = [yq1(1:end-1)];
    b= {P};
    xAGV = 23.8;
    yAGV = 18.7;
    aAGV = 0;
    endIDindex = length(y);

    bwd = 1; 

elseif strcmp(name, 'EWN_6')

    P.x=[27.5 24 23.8];
    P.y=[2.1 2.1 2.1];
    n=1000;
    [xq1, yq1] = bezier3(P.x,P.y,n);

    x = [xq1(1:end-1)];
    y = [yq1(1:end-1)];
    b= {P};
    xAGV = 23.8;
    yAGV = 18.7;
    aAGV = 0;
    endIDindex = length(y);

elseif strcmp(name, 'EWN_7')

    P.x=[23.8 23.8 23.8];
    P.y=[2.1 4 5];
    n=1000;
    [xq1, yq1] = bezier3(P.x,P.y,n);

    P1.x=[23.8 23.8 22];
    P1.y=[5 7 7];
    [xq2, yq2] = bezier3(P1.x,P1.y,n);

    P2.x=[22 10 3.42];
    P2.y=[7 7 7];
    [xq3, yq3] = bezier3(P2.x,P2.y,n);

    x = [xq1(1:end-1), xq2(1:end-1), xq3(1:end-1)];
    y = [yq1(1:end-1), yq2(1:end-1), yq3(1:end-1)];
    b= {P, P1, P2};%, P3, P4, P5};
    xAGV = 3.42;
    yAGV = 7;
    aAGV = 0;
    endIDindex = length(y);

elseif strcmp(name, 'EWN_8')

    P.x=[3.42 3.42 3.42];
    P.y=[7 8 9];
    n=1000;
    [xq1, yq1] = bezier3(P.x,P.y,n);

    bwd = 1;

    x = [xq1(1:end-1)];
    y = [yq1(1:end-1)];
    b= {P};
    xAGV = 3.42;
    yAGV = 7;
    aAGV = 0;
    endIDindex = length(y);

elseif strcmp(name, 'EWN_9')

    P.x=[3.42 3.42 3.42];
    P.y=[9 8 7];
    n=1000;
    [xq1, yq1] = bezier3(P.x,P.y,n);

    x = [xq1(1:end-1)];
    y = [yq1(1:end-1)];
    b= {P};
    xAGV = 3.42;
    yAGV = 7;
    aAGV = 0;
    endIDindex = length(y);
end

% Abstände zw den Punkten w
w = zeros(1,length(x)-1);

for i= 1:length(x)-1
    w(i) = sqrt ( (x(i+1) - x(i))^2 + (y(i+1) - y(i))^2 );
end



end
