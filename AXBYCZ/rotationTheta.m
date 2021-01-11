function theta = rotationTheta( g )
%rotationTheta rotation angle of a homogeneous transformation
%
%   theta = rotationTheta (g)
%   g:      Homogeneous transformation, 4 x 4
%   theta:  Rotation angle

tr=(trace(g(1:3,1:3))-1)/2;

if tr>1

    tr=1;

elseif tr<-1

    tr=-1;

end

theta=acos(tr);

end

