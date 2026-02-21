function [rpm] = LSimoveC_UC_Kin_Mecanum_V2W(v, positionFromRoot, wheelRadius) 


rpm(1) = 1/wheelRadius * ( v(1) - v(2) - (abs(positionFromRoot(1).y) + abs(positionFromRoot(1).x))*v(3) ); 
rpm(2) = 1/wheelRadius * ( v(1) + v(2) + (abs(positionFromRoot(2).y) + abs(positionFromRoot(2).x))*v(3) ); 
rpm(3) = 1/wheelRadius * ( v(1) + v(2) - (abs(positionFromRoot(3).y) + abs(positionFromRoot(3).x))*v(3) ); 
rpm(4) = 1/wheelRadius * ( v(1) - v(2) + (abs(positionFromRoot(4).y) + abs(positionFromRoot(4).x))*v(3) ); 

