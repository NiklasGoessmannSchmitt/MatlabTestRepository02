function ScannerData3D(dAt)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
%ScannerData3D(dAt_Matrix)
figure;
title('ScannerData3D')
xlabel('X');
ylabel('Y');
% g.curMapPos(1)=0;
% g.curMapPos(2)=0;
% g.curMapPos(3)=0;
% plotAGV(a,g)


for i=1:length(dAt)                 %667
    for j=1:length(dAt(1).theta)    %550 Points
        [x, y]=pol2cart(dAt(i).theta(j), dAt(i).rho(j));
        x_scan(i, j)=x;            %??????
        y_scan(i,j)=y;
    end
end

z_scan= (1: length(dAt))';
zz= repmat(z_scan, 1, length(dAt(1).theta));
mesh(x_scan,y_scan,zz);
hold on;

end