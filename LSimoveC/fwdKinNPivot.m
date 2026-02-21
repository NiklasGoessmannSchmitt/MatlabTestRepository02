function v = fwdKinNPivot(vs, alpha, pivotParam)

x = vs.*0;
y = vs.*0;

[rows, ~] = size(vs);
for n = 1:rows

    x(n,:) = vs(n,:) .* cos(alpha(n,:));
    y(n,:) = vs(n,:) .* sin(alpha(n,:));

end

v(1,:) = mean(x) * 2*pi * pivotParam(1).diameter/2 / 60 / pivotParam(1).gearratio;
v(2,:) = mean(y) * 2*pi * pivotParam(1).diameter/2 / 60 / pivotParam(1).gearratio;
v(3,:) = (y(1,:) * abs(pivotParam(1).position.x) ...
         -y(2,:) * abs(pivotParam(2).position.x)) / length(pivotParam) ...
                   * 2*pi * pivotParam(1).diameter/2 / 60 / pivotParam(1).gearratio;

end
