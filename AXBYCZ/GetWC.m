function [ WC ] = GetWC( qC )
%refer to eq. (9)
%   GetWC calculates Wc.
%   Wc is a 4*16 matrix.

qC0 = qC(1); qC1 = qC(2); qC2 = qC(3); qC3 = qC(4);
WC = [qC0,     -qC1,    -qC2,    -qC3,    -qC1,    -qC0,    qC3,     -qC2,    -qC2,    -qC3,    -qC0,    qC1,     -qC3,    qC2,     -qC1,    -qC0;
     qC1,     qC0,     -qC3,    qC2,     qC0,     -qC1,    -qC2,    -qC3,    qC3,     -qC2,    qC1,     qC0,     -qC2,    -qC3,    -qC0,    qC1;
     qC2,     qC3,     qC0,     -qC1,    -qC3,    qC2,     -qC1,    -qC0,    qC0,     -qC1,    -qC2,    -qC3,    qC1,     qC0,     -qC3,    qC2;
     qC3,     -qC2,    qC1,     qC0,     qC2,     qC3,     qC0,     -qC1,    -qC1,    -qC0,    qC3,     -qC2,    qC0,     -qC1,    -qC2,    -qC3 ];

end

