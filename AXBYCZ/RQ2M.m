function [ M ] = RQ2M( q )
%Right Multiplier Matrix from a quaternion
%   Detailed explanation goes here
q0 = q(1); qx = q(2); qy = q(3); qz = q(4);
M = [q0, -qx, -qy, -qz;
    qx,  q0,  qz,  -qy;
    qy,  -qz, q0,  qx;
    qz,  qy,  -qx, q0];

end