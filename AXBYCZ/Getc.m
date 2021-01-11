function c = Getc( RA_noise,RB_noise,RC_noise,RX_init,RY_init,RZ_init )
%refer to eq. (29)
%   Getc calculates c vector for Fr=c.

M=size(RA_noise,3); %number of measurment configurations
c=zeros(9*M,1);

for i=1:M
    RA=RA_noise(:,:,i); 
    RB=RB_noise(:,:,i); 
    RC=RC_noise(:,:,i);
    RAXBYCZ=-RA*RX_init*RB + RY_init*RC*RZ_init;
    
    c(9*i-8:9*i)=reshape(RAXBYCZ,[9,1]);
end

end

