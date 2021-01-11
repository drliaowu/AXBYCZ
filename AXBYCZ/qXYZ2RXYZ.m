function [RX0, RY0, RZ0] = qXYZ2RXYZ (V)
% obtain RX0, RY0, RZ0 from V

qX = V(1:4,1)/norm(V(1:4,1));

qYZ = V(5:20,1)/norm(V(1:4,1));

RX0 = Q2R(qX);

qYZ = qYZ*sign(qYZ(1)); %make the sign of qY0*qZ0 plus

SignY0 = 1; SignY1 = sign(qYZ(5)); SignY2 = sign(qYZ(9)); SignY3 = sign(qYZ(13));
qY0 = norm(qYZ(1:4))*SignY0; qY1 = norm(qYZ(5:8))*SignY1; qY2 = norm(qYZ(9:12))*SignY2; qY3 = norm(qYZ(13:16))*SignY3;
qY = [qY0;qY1;qY2;qY3]; qY = qY/norm(qY);

SignZ0 = 1; SignZ1 = sign(qYZ(2)); SignZ2 = sign(qYZ(3)); SignZ3 = sign(qYZ(4));
qZ0 = norm(qYZ([1,5,9,13]))*SignZ0; qZ1 = norm(qYZ([2,6,10,14]))*SignZ1; qZ2 = norm(qYZ([3,7,11,15]))*SignZ2; qZ3 = norm(qYZ([4,8,12,16]))*SignZ3;
qZ = [qZ0;qZ1;qZ2;qZ3]; qZ = qZ/norm(qZ);

RY0 = Q2R(qY);

RZ0 = Q2R(qZ);
end