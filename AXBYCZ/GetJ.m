function [ J ] = GetJ( RA_noise, RY_sln, RC_noise )
%refer to eq. (38)
%   GetJ calculates J matrix for Jt=p.

M = size(RA_noise,3); % number of measurement configurations
J = zeros(3*M,9);

for i=1:M
    J(3*i-2:3*i,:)=[RA_noise(:,:,i),-eye(3),-RY_sln*RC_noise(:,:,i)];
end
end

