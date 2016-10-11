function out=clockToString(t)
% converts a clock vector into a nice string
%% CHANGES
%   VI063009A: Support formatted display of milliseconds -- Vijay Iyer 6/30/09
%
%% **********************************************************

t(1:5)=fix(t(1:5)); %VI063009A
out=sprintf('%d/%d/%d %d:', t(2), t(3), t(1), t(4));
if t(5)<10
    out=[out sprintf('0%d:', t(5))];
else
    out=[out sprintf('%d:', t(5))];
end
out = [out sprintf('%06.3f', t(6))];
%%%VI063009A: Removed%%%%%%%%
%     if t(6)<10
%         out=[out sprintf('0%d', t(6))];
%     else
%         out=[out sprintf('%d', t(6))];
%     end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
