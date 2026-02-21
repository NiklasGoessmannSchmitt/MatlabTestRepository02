function r = frameRotate(s, rot)

r(1) = s(1) * cos(rot) - s(2) * sin(rot);
r(2) = s(1) * sin(rot) + s(2) * cos(rot); 
r(3) = s(3); 