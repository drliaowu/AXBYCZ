% Please refer to "Liao Wu, Jiaole Wang, Lin Qi, Keyu Wu, Hongliang Ren, Max Q.-H. Meng. 
% Simultaneous hand-eye, tool-flange and robot-robot calibration for comanipulation by solving the AXB = YCZ problem. 
% IEEE Transactions on Robotics. 2016, 32(2): 413-428. "

function [RX0, RY0, RZ0 ] = FindInitialXYZ( RA, RB, RC )
%FindInitialXYZ Initial solution of AXB=YCZ
% 
%   [RX0, RY0, RZ0 ] = FindInitialXYZ( RA, RB, RC )
%   RA, RB, RC:                 Rotation matrix, 3*3*N
%   RX0, RY0, RZ0:              Initial rotation matrix, 3*3

N_motion = size(RA,3);

M_data = min(N_motion,10);
for i=1:M_data
    WAB(4*i-3:4*i,1:4) = RQ2M(R2Q(RB(1:3,1:3,i)))*LQ2M(R2Q(RA(1:3,1:3,i)));
    WC(4*i-3:4*i,1:16) = GetWC(R2Q(RC(1:3,1:3,i)));
end

lamda_min1=inf;
lamda_min2=inf; 
lamda_min3=inf;
lamda_min4=inf;
lamda_min5=inf;%lamda_min1<lamda_min2<lamda_min3;
V1=[]; V2=[]; V3=[]; V4=[]; V5=[];

for j=0:1:2^(M_data-1)-1
    WABC = [WAB,-WC];
    for k=1:M_data
        WABC(4*k-3:4*k,1:4) = WABC(4*k-3:4*k,1:4)*(2*bitget(j,k)-1);
    end
    [Vtemp,Dtemp] = eig(WABC'*WABC);
    if Dtemp(1,1)<lamda_min1
        lamda_min5 = lamda_min4;
        V5 = V4;
        
        lamda_min4 = lamda_min3;
        V4 = V3;
        
        lamda_min3 = lamda_min2;
        V3 = V2;
        
        lamda_min2 = lamda_min1;
        V2 = V1;
        
        lamda_min1 = Dtemp(1,1);
        V1 = Vtemp;
        
    elseif Dtemp(1,1)<lamda_min2
        lamda_min5 = lamda_min4;
        V5 = V4;
        
        lamda_min4 = lamda_min3;
        V4 = V3;
        
        lamda_min3 = lamda_min2;
        V3 = V2;
        
        lamda_min2 = Dtemp(1,1);
        V2 = Vtemp;
        
    elseif Dtemp(1,1)<lamda_min3
        lamda_min5 = lamda_min4;
        V5 = V4;
        
        lamda_min4 = lamda_min3;
        V4 = V3;
        
        lamda_min3 = Dtemp(1,1);
        V3 = Vtemp;
        
    elseif Dtemp(1,1)<lamda_min4
        lamda_min5 = lamda_min4;
        V5 = V4;
        
        lamda_min4 = Dtemp(1,1);
        V4 = Vtemp;
    
    elseif Dtemp(1,1)<lamda_min5
        lamda_min5 = Dtemp(1,1);
        V5 = Vtemp;
    end

end
[RX01, RY01, RZ01] = qXYZ2RXYZ(V1);
[RX02, RY02, RZ02] = qXYZ2RXYZ(V2);
[RX03, RY03, RZ03] = qXYZ2RXYZ(V3);
[RX04, RY04, RZ04] = qXYZ2RXYZ(V4);
[RX05, RY05, RZ05] = qXYZ2RXYZ(V5);

err1 =0;
err2= 0;
err3= 0;
err4=0;
err5=0;
for i=1:M_data
    err1=err1 + norm(vlogR(RA(1:3,1:3,i)*RX01*RB(1:3,1:3,i)*transpose(RY01*RC(1:3,1:3,i)*RZ01)));
    err2=err2 + norm(vlogR(RA(1:3,1:3,i)*RX02*RB(1:3,1:3,i)*transpose(RY02*RC(1:3,1:3,i)*RZ02)));
    err3=err3 + norm(vlogR(RA(1:3,1:3,i)*RX03*RB(1:3,1:3,i)*transpose(RY03*RC(1:3,1:3,i)*RZ03)));
    err4=err4 + norm(vlogR(RA(1:3,1:3,i)*RX04*RB(1:3,1:3,i)*transpose(RY04*RC(1:3,1:3,i)*RZ04)));
    err5=err5 + norm(vlogR(RA(1:3,1:3,i)*RX05*RB(1:3,1:3,i)*transpose(RY05*RC(1:3,1:3,i)*RZ05)));
end
errmin = min([err1,err2,err3,err4,err5]);
if err1 == errmin
    RX0 = RX01; RY0 =RY01; RZ0 = RZ01;
elseif err2 == errmin
    RX0 = RX02; RY0 = RY02; RZ0 = RZ02;
elseif err3 == errmin
    RX0 = RX03; RY0 = RY03; RZ0 = RZ03; 
elseif err4 == errmin
    RX0 = RX04; RY0 = RY04; RZ0 = RZ04;
elseif err5 == errmin
    RX0 = RX05; RY0 = RY05; RZ0 = RZ05; 
end

end

