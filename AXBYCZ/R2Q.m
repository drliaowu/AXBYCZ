function [ Q ] = R2Q( R )
%Rotation Matrix to Quaternion
% theta = rotationTheta(R);
% w = rotationW(R,theta);
% 
% Q=[cos(theta/2);w*sin(theta/2)];

Qxx = R(1,1); Qyy = R(2,2); Qzz = R(3,3); Qzy = R(3,2); Qyz = R(2,3); Qxz = R(1,3); Qzx = R(3,1); Qyx = R(2,1); Qxy = R(1,2);

t = Qxx+Qyy+Qzz;

if (t>=0)
r = sqrt(1+t);
s = 0.5/r;
w = 0.5*r;
x = (Qzy-Qyz)*s;
y = (Qxz-Qzx)*s;
z = (Qyx-Qxy)*s;

elseif (abs(Qxx) >= max(abs(Qyy),abs(Qzz)))
r = sqrt(1+Qxx-Qyy-Qzz);
s = 0.5/r;
w = (Qzy-Qyz)*s;
x = 0.5*r;
y = (Qxy+Qyx)*s;
z = (Qzx+Qxz)*s;

elseif (abs(Qyy) >= max(abs(Qxx),abs(Qzz)))
r = sqrt(1-Qxx+Qyy-Qzz);
s = 0.5/r;
w = (Qxz-Qzx)*s;
x = (Qxy+Qyx)*s;
y = 0.5*r;
z = (Qyz+Qzy)*s;

else
r = sqrt(1-Qxx-Qyy+Qzz);
s = 0.5/r;
w = (Qyx-Qxy)*s;
x = (Qxz+Qzx)*s;
y = (Qyz+Qzy)*s;
z = 0.5*r;

end
Q = [w;x;y;z];
end

