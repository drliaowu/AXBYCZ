% Please refer to "Liao Wu, Jiaole Wang, Lin Qi, Keyu Wu, Hongliang Ren, Max Q.-H. Meng. 
% Simultaneous hand-eye, tool-flange and robot-robot calibration for comanipulation by solving the AXB = YCZ problem. 
% IEEE Transactions on Robotics. 2016, 32(2): 413-428. "

function [gX,gY,gZ,errR,errt,iter]=AXBYCZ(gA,gB,gC, RX_init, RY_init, RZ_init)
%AXBYCZ Solving gX, gY, gZ from AXB=YCZ given gA, gB, gC
% 
%   [gX,gY,gZ,errR,errt,iter]=AXBYCZ(gA,gB,gC, RX_init, RY_init, RZ_init)
%   gA, gB, gC:                 Homogeneous transformation, 4*4*N
%   RX_init, RY_init, RZ_init:  Initial rotation matrix, 3*3
%   gX, gY, gZ:                 Homogeneous transformation, 4*4
%   errR:                       Rotational error, N*1 
%   errt:                       Translational error, N*1 
%   iter:                       Iterations, scaler

%% load data
N = size(gA,3);

RA = gA(1:3,1:3,:);
RB = gB(1:3,1:3,:);
RC = gC(1:3,1:3,:);

tA = reshape(gA(1:3,4,:),[3,N]);
tB = reshape(gB(1:3,4,:),[3,N]);
tC = reshape(gC(1:3,4,:),[3,N]);

%% Solve RX,RY,RZ
r=inf;

iter=0;

while (norm(r)>1e-10)
    % eq. (29)
    F=GetF(RA,RB,RC,RX_init,RY_init,RZ_init);
    c=Getc(RA,RB,RC,RX_init,RY_init,RZ_init);
    r=F\c;
    
    % Update
    RX_init = rotationMatrix(r(1:3)/norm(r(1:3)),norm(r(1:3)))*RX_init;
    RY_init = rotationMatrix(r(4:6)/norm(r(4:6)),norm(r(4:6)))*RY_init;
    RZ_init = rotationMatrix(r(7:9)/norm(r(7:9)),norm(r(7:9)))*RZ_init;
    
    iter = iter+1;
    if iter>=100
        break;
    end
    
end

RX_sln = RX_init;
RY_sln = RY_init;
RZ_sln = RZ_init;

%% Solve tX, tY, tZ
% eq. (38)
J = GetJ(RA, RY_sln, RC);
p = Getp(RA, RX_sln, RY_sln, tA, tB, tC);

t_sln=J\p;
tX_sln=t_sln(1:3);
tY_sln=t_sln(4:6);
tZ_sln=t_sln(7:9);

gX = [RX_sln,tX_sln;
    0,0,0,1];
gY = [RY_sln,tY_sln;
    0,0,0,1];
gZ = [RZ_sln,tZ_sln;
    0,0,0,1];

errR = zeros(N,1);
errt = zeros(N,1);

for i=1:N
    errg=gA(:,:,i)*gX*gB(:,:,i)\(gY*gC(:,:,i)*gZ);
    errR(i) = norm(vlogR(errg(1:3,1:3)));
    errt(i) = norm(errg(1:3,4));
end
