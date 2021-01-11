% Please refer to "Liao Wu, Jiaole Wang, Lin Qi, Keyu Wu, Hongliang Ren, Max Q.-H. Meng. 
% Simultaneous hand-eye, tool-flange and robot-robot calibration for comanipulation by solving the AXB = YCZ problem. 
% IEEE Transactions on Robotics. 2016, 32(2): 413-428. "

%% Add path
% http://petercorke.com/wordpress/Toolboxes

addpath(genpath('robot'));
addpath(genpath('common'));

%% Generate data 
% Rotation matrix of unknowns: X, Y, Z
clear;
clc;

SimulationCycle=10; % number of simulation cycles

RotationErrX=zeros(SimulationCycle,1);
RotationErrY=zeros(SimulationCycle,1);
RotationErrZ=zeros(SimulationCycle,1);
TranslationErrX=zeros(SimulationCycle,1);
TranslationErrY=zeros(SimulationCycle,1);
TranslationErrZ=zeros(SimulationCycle,1);

ErrMatrix = zeros(19,6);

j=0;

for N_motion=[10,20,30,40,50,60,70,80,90,100]

    j=j+1;

    for k=1:SimulationCycle

        RX_true = rotz(pi/2+0.01); % unit is radian
        RY_true = rotz(pi-0.02);
        RZ_true = rotz(pi/4+0.01);

        tX_true = [0;0;0.200-0.003];
        tY_true = [2.000+0.010;0;0];
        tZ_true = [0;0;0.100+0.002];

        % Rotation matrix of knowns: A, C, B

%         % small noise
%         RotationNoiseScale_A = 0.025/180*pi; % unit:rad
%         RotationNoiseScale_B = 0.05/180*pi; % unit:rad
%         RotationNoiseScale_C = 0.025/180*pi; % unit:rad
% 
%         TranslationNoiseScale_A = 0.1/1000; % unit:m
%         TranslationNoiseScale_B = 0.2/1000; % unit:m
%         TranslationNoiseScale_C = 0.1/1000; % unit:m
% 
%         % medium noise
%         RotationNoiseScale_A = 0.1/180*pi; % unit:rad
%         RotationNoiseScale_B = 0.2/180*pi; % unit:rad
%         RotationNoiseScale_C = 0.1/180*pi; % unit:rad
% 
%         TranslationNoiseScale_A = 0.5/1000; % unit:m
%         TranslationNoiseScale_B = 1.0/1000; % unit:m
%         TranslationNoiseScale_C = 0.5/1000; % unit:m

        % big noise
        RotationNoiseScale_A = 0.25/180*pi; % unit:rad
        RotationNoiseScale_B = 0.5/180*pi; % unit:rad
        RotationNoiseScale_C = 0.25/180*pi; % unit:rad

        TranslationNoiseScale_A = 1.0/1000; % unit:m
        TranslationNoiseScale_B = 2.0/1000; % unit:m
        TranslationNoiseScale_C = 1.0/1000; % unit:m

        RA_true = zeros(3,3,N_motion);
        RB_true = zeros(3,3,N_motion);
        RC_true = zeros(3,3,N_motion);
        RA_noise =zeros(3,3,N_motion);
        RB_noise =zeros(3,3,N_motion);
        RC_noise =zeros(3,3,N_motion);

        tA_true = zeros(3,N_motion);
        tB_true = zeros(3,N_motion);
        tC_true = zeros(3,N_motion);
        tA_noise =zeros(3,N_motion);
        tB_noise =zeros(3,N_motion);
        tC_noise =zeros(3,N_motion);

        gA_true = zeros(4,4,N_motion);
        gC_true = zeros(4,4,N_motion);

        gA_noise = zeros(4,4,N_motion);
        gB_noise = zeros(4,4,N_motion);
        gC_noise = zeros(4,4,N_motion);

        QA_true = zeros(N_motion,6);
        QC_true = zeros(N_motion,6);

        mdl_puma560; %generate a puma 560 robot: p560

        gA_standard = fkine(p560, [0,pi/3,-5*pi/6,0,-pi/4,0]);
        RA_standard = gA_standard(1:3,1:3);
        tA_standard = gA_standard(1:3,4);

        gC_standard = fkine(p560, [0,-pi/3,-1*pi/6,0,pi/4,0]);
        RC_standard = gC_standard(1:3,1:3);
        tC_standard = gC_standard(1:3,4);

    for iter = 1:N_motion
        % generate gA,gB,gC nearby gA_standard, gB_standard, gC_standard

        RA_true(:,:,iter) = RA_standard*rotx(pi/4-pi/2*rand(1))*roty(pi/4-pi/2*rand(1))*rotz(pi-pi*2*rand(1));
        gA_true(:,:,iter) = [RA_true(:,:,iter),tA_standard;0,0,0,1];

        RC_true(:,:,iter) = RC_standard*rotx(pi/4-pi/2*rand(1))*roty(pi/4-pi/2*rand(1))*rotz(pi-pi*2*rand(1));
        gC_true(:,:,iter) = [RC_true(:,:,iter),tC_standard;0,0,0,1];

        RB_true(:,:,iter) = RX_true'*RA_true(:,:,iter)'*RY_true*RC_true(:,:,iter)*RZ_true;

        % Add noise to RA,RB,RC
        RotationAxisA_noise = 1-2*rand(3,1); RotationAxisA_noise = RotationAxisA_noise / norm(RotationAxisA_noise);
        RotationAngleA_noise = RotationNoiseScale_A*rand(1);
        
        RotationAxisB_noise = 1-2*rand(3,1); RotationAxisB_noise = RotationAxisB_noise / norm(RotationAxisB_noise);
        RotationAngleB_noise = RotationNoiseScale_B*rand(1);
        
        RotationAxisC_noise = 1-2*rand(3,1); RotationAxisC_noise = RotationAxisC_noise / norm(RotationAxisC_noise);
        RotationAngleC_noise = RotationNoiseScale_C*rand(1);

        RA_noise(:,:,iter) = RA_true(:,:,iter)*rotationMatrix(RotationAxisA_noise,RotationAngleA_noise);
        RB_noise(:,:,iter) = RB_true(:,:,iter)*rotationMatrix(RotationAxisB_noise,RotationAngleB_noise);
        RC_noise(:,:,iter) = RC_true(:,:,iter)*rotationMatrix(RotationAxisC_noise,RotationAngleC_noise);

        %generate tA,tB,tC 
        tA_true(:,iter) = gA_true(1:3,4,iter);
        tC_true(:,iter) = gC_true(1:3,4,iter);
        tB_true(:,iter) = RX_true'*RA_true(:,:,iter)'*(RY_true*RC_true(:,:,iter)*tZ_true + RY_true*tC_true(:,iter) + tY_true - RA_true(:,:,iter)*tX_true - tA_true(:,iter));

        % Add noise to tA,tB,tC
        TransDirA_noise = 1-2*rand(3,1); TransDirA_noise = TransDirA_noise / norm(TransDirA_noise);
        TransDirB_noise = 1-2*rand(3,1); TransDirB_noise = TransDirB_noise / norm(TransDirB_noise);
        TransDirC_noise = 1-2*rand(3,1); TransDirC_noise = TransDirC_noise / norm(TransDirC_noise);
 
        TransAmpA_noise = TranslationNoiseScale_A*rand(1);
        TransAmpB_noise = TranslationNoiseScale_B*rand(1);
        TransAmpC_noise = TranslationNoiseScale_C*rand(1);

        tA_noise(:,iter) = tA_true(:,iter)+TransAmpA_noise*TransDirA_noise;
        tB_noise(:,iter) = tB_true(:,iter)+TransAmpB_noise*TransDirB_noise;
        tC_noise(:,iter) = tC_true(:,iter)+TransAmpC_noise*TransDirC_noise;
  

        gA_noise(:,:,iter) = [RA_noise(:,:,iter),tA_noise(:,iter);
                                0,0,0,1];
        gB_noise(:,:,iter) = [RB_noise(:,:,iter),tB_noise(:,iter);
                                0,0,0,1];
        gC_noise(:,:,iter) = [RC_noise(:,:,iter),tC_noise(:,iter);
                                0,0,0,1];
    end


    [RX_init, RY_init, RZ_init] = FindInitialXYZ(RA_noise, RB_noise, RC_noise);


    [gX,gY,gZ,errR,errt,iteration]=AXBYCZ(gA_noise,gB_noise,gC_noise,RX_init,RY_init,RZ_init);
 
    RX_sln = gX(1:3,1:3);
    RY_sln = gY(1:3,1:3);
    RZ_sln = gZ(1:3,1:3);

    tX_sln = gX(1:3,4);
    tY_sln = gY(1:3,4);
    tZ_sln = gZ(1:3,4);

    %% Evaluation
    RotationErrX(k) = rotationTheta((RX_sln/RX_true))/pi*180;
    RotationErrY(k) = rotationTheta((RY_sln/RY_true))/pi*180;
    RotationErrZ(k) = rotationTheta((RZ_sln/RZ_true))/pi*180;

    TranslationErrX(k) = norm(tX_sln-tX_true)*1000;
    TranslationErrY(k) = norm(tY_sln-tY_true)*1000;
    TranslationErrZ(k) = norm(tZ_sln-tZ_true)*1000;

    %disp(k)
 
    end

    MaxRotationErrX = max(RotationErrX);
    MaxRotationErrY = max(RotationErrY);
    MaxRotationErrZ = max(RotationErrZ);
    MaxTranslationErrX = max(TranslationErrX);
    MaxTranslationErrY = max(TranslationErrY);
    MaxTranslationErrZ = max(TranslationErrZ);

    MeanRotationErrX = mean(RotationErrX);
    MeanRotationErrY = mean(RotationErrY);
    MeanRotationErrZ = mean(RotationErrZ);
    MeanTranslationErrX = mean(TranslationErrX);
    MeanTranslationErrY = mean(TranslationErrY);
    MeanTranslationErrZ = mean(TranslationErrZ);

    StdRotationErrX = std(RotationErrX);
    StdRotationErrY = std(RotationErrY);
    StdRotationErrZ = std(RotationErrZ);
    StdTranslationErrX = std(TranslationErrX);
    StdTranslationErrY = std(TranslationErrY);
    StdTranslationErrZ = std(TranslationErrZ);

    ErrMatrix(:,j) = [N_motion;MaxRotationErrX;MaxRotationErrY;MaxRotationErrZ;MaxTranslationErrX;MaxTranslationErrY;MaxTranslationErrZ;...
        MeanRotationErrX;MeanRotationErrY;MeanRotationErrZ;MeanTranslationErrX;MeanTranslationErrY;MeanTranslationErrZ;...
        StdRotationErrX;StdRotationErrY;StdRotationErrZ;StdTranslationErrX;StdTranslationErrY;StdTranslationErrZ];

    disp(j)

end

figure(1)
plot(ErrMatrix(1,:),ErrMatrix(8,:))% Rotational Error, X
figure(2)
plot(ErrMatrix(1,:),ErrMatrix(9,:))% Rotational Error, Y
figure(3)
plot(ErrMatrix(1,:),ErrMatrix(10,:))% Rotational Error, Z
figure(4)
plot(ErrMatrix(1,:),ErrMatrix(11,:))% Translational Error, X
figure(5)
plot(ErrMatrix(1,:),ErrMatrix(12,:))% Translational Error, Y
figure(6)
plot(ErrMatrix(1,:),ErrMatrix(13,:))% Translational Error, Z