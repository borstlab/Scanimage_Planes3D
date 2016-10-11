function C = xcorr2circ(A,B)
%circular cross correlation
C = fftshift(ifft2(fft2(single(A)).*conj(fft2(single(B)))));
end



%--------------------------------------------------------------------------%
% xcorr2circ.m                                                             %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
