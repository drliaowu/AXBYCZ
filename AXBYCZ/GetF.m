function F = GetF( RA_noise,RB_noise,RC_noise,RX_init,RY_init,RZ_init )
%refer to eq. (29)
%   GetF calculates F matrix for Fr=c.

M=size(RA_noise,3); %the number of measurement configurations
F=zeros(9*M,9);

for i=1:M
    RA = RA_noise(:,:,i); 
    RB = RB_noise(:,:,i); 
    RC = RC_noise(:,:,i);
    RXB = RX_init*RB; 
    RYCZ = RY_init*RC*RZ_init; 
    
    F(9*i-8:9*i-6,:) = [ -RA*skew(RXB(:,1)), skew(RYCZ(:,1)), RY_init*RC*skew(RZ_init(:,1)) ];
    F(9*i-5:9*i-3,:) = [ -RA*skew(RXB(:,2)), skew(RYCZ(:,2)), RY_init*RC*skew(RZ_init(:,2))];
    F(9*i-2:9*i,:) = [ -RA*skew(RXB(:,3)), skew(RYCZ(:,3)), RY_init*RC*skew(RZ_init(:,3))];
end

end

