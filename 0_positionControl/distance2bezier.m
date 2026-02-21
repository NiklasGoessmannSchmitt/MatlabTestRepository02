%%Simu getTrack mit bezier Punkten (Bsp s curve)

radius =3 ;

P.x=[-2 -2 0];
P.y=[0   0 0];

P1.x=[0 radius radius];
P1.y=[0   0    radius];

P2.x=[radius   radius 2*radius];
P2.y=[radius 2*radius 2*radius];

P3.x=[2*radius 2*radius 2*radius+3];
P3.y=[2*radius   2*radius 2*radius];

b= {P, P1, P2, P3};


%% distance2bezier3
for j = 1:5000
xAGV = 5.9;
yAGV = 6;
x = cell(1,length(b));
y = cell (1,length(b));
distf = cell (1,length(b));
%dista = ones(1,length(b));
pick= ones(1,length(b));
%t = ones(1,length(b));
%options = optimset('MaxIter',200,'TolX',0.01);%,'Display','iter');


for i= 1:length(b)
    
     %    x{i}=@(t) (1-t).^2*b{i}.x(1) + 2*(1-t).*t*b{i}.x(2) + t.^2*b{i}.x(3);
      %   y{i}=@(t) (1-t).^2*b{i}.y(1) + 2*(1-t).*t*b{i}.y(2) + t.^2*b{i}.y(3);

        distf{i}=@(t) sqrt((((1-t).^2*b{i}.x(1) + 2*(1-t).*t*b{i}.x(2) + t.^2*b{i}.x(3))-xAGV)^2 + (((1-t).^2*b{i}.y(1) + 2*(1-t).*t*b{i}.y(2) + t.^2*b{i}.y(3)) - yAGV)^2);
        pick(i)=distf{i}(0)+distf{i}(0.5)+distf{i}(1);
     %   [t(i),dista(i)]=fminbnd(distf{i},0,1,options);
     dista(i)=100;
end
[~, q] = min(pick);
pick(q)=[];
[~, r] = min(pick);

if q > r
    s =-1;
end

for i=q:s:r
    
    [t(i),dista(i)]=fminbnd(distf{i},0,1);

end

[~, q] = min(dista);


%[~, w] = min(dista);
t=t(q);
X=(1-t)^2*b{q}.x(1) + 2*(1-t)*t*b{q}.x(2) + t^2*b{q}.x(3);
Y=(1-t)^2*b{q}.y(1) + 2*(1-t)*t*b{q}.y(2) + t^2*b{q}.y(3);
end

%%
% a=b{i}.x(1); h=b{i}.x(2); c=b{i}.x(3); d=b{i}.y(1); e=b{i}.y(2); f=b{i}.y(3);
% g=@(t) (2.*(2.*f.*t-2.*e.*t+2.*d.*(t-1)-2.*e.*(t-1)).*(f.*t.^2-2.*e.*(t-1).*t+d.*(t-1).^2-yAGV)+2.*(2.*c.*t-2.*h.*t-2.*h.*(t-1)+2.*a.*(t-1)).*(c.*t.^2-2.*h.*(t-1).*t+a.*(t-1).^2-xAGV))./(2.*sqrt((f.*t.^2-2.*e.*(t-1).*t+d.*(t-1).^2-yAGV).^2+(c.*t.^2-2.*h.*(t-1).*t+a.*(t-1).^2-xAGV).^2));

%% plot
% t=0:0.001:1;
% for i = 1:length(x)
%     plot(x{i}(t),y{i}(t))
%     hold on
% end
% 
% scatter(xAGV,yAGV,'g')
% scatter(X,Y,'r')
% 
% hold off
% %%
% a=-1;
% for j=3:a:2
%     dista(j) = j;
% end
% min(dista)