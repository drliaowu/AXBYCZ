function w = vlogR( R )
%vlogR Logarithm mapping from a rotation matrix to a twist
%
%   w = vlogR (R)
%   R:      Rotation matrix
%   w:      twist, 3 x 1

DELTA=10^(-12);

tr=(trace(R)-1)/2;

if tr>1

    tr=1;

elseif tr<-1

    tr=-1;

end

fai=acos(tr);

if abs(fai)<DELTA

    w=[0;0;0];

elseif abs(fai-pi)<DELTA

    warning('Logarithm of rotation matrix with angle PI.' );
    
    [V,D]=eig(R,'vector');
    
    D = real(D);
    
    V = real(V);
    
    if max(D)==D(1)
    
        w=V(:,1);
    
    elseif max(D)==D(2)
    
        w=V(:,2);
    
    else
        
        w=V(:,3);
    
    end
    
    if max(max(cos(fai)*eye(3)+(1-cos(fai))*(w*w')+sin(fai)*skew(w)-R))>DELTA
    
        w=-w;
    
    end
    
    w = w*fai;

else
    
    w=fai/(2*sin(fai))*[R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)];

end

end

