function [ p ] = Getp( RA_noise, RX_sln, RY_sln, tA_noise, tB_noise, tC_noise )
%refer to eq. (38)
%   Getp calculates p vector for Jt=p.

M = size(RA_noise,3); % number of measurement configurations
p = zeros(3*M,1);

for i=1:M
    p(3*i-2:3*i,1)=-tA_noise(:,i) - RA_noise(:,:,i)*RX_sln*tB_noise(:,i) + RY_sln*tC_noise(:,i);
end
end

