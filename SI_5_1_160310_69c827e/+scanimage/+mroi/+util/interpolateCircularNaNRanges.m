function data = interpolateCircularNaNRanges(data,bounds,algorithm)

if nargin < 3 || isempty(algorithm)
    algorithm = 'tryall';
end

% data needs to be a column vector
data = data(:);
bounds = sort(bounds);

nanRanges = scanimage.mroi.util.findNaNRanges(data);
if isempty(nanRanges);return;end % Nothing to interpolate

if isnan(data(1)) && isnan(data(end))
    shifted = nanRanges(end,2)-nanRanges(end,1)+1;
    data = circshift(data,shifted);
    nanRanges = scanimage.mroi.util.findNaNRanges(data);
else
    shifted = 0;
end

datalen = length(data);

for i = 1:size(nanRanges,1)
    st = nanRanges(i,1); % start index
    en = nanRanges(i,2); % end index
    len = en - st + 1;
    
    p  = st - 1;
    pp = st - 2;
    n  = en + 1;
    nn = en + 2;
    
    % ensure index rolls over correctly
    p  = p  + datalen*(p<1);
    pp = pp + datalen*(pp<1);
    n  = n  - datalen*(n>datalen);
    nn = nn - datalen*(nn>datalen);
    
    % fetch data
    p  = data(p);
    pp = data(pp);
    n  = data(n);
    nn = data(nn);
    
    assert(~(isnan(p)||isnan(n)||isnan(pp)||isnan(nn)),'Something bad happened');
    
    switch algorithm
        case {'tryall','default'}
            data(st:en) = interpolateNaNRangeDefault(pp,p,n,nn,len,bounds);
        case 'spline'
            data(st:en) = interpolateNaNRangeSpline(pp,p,n,nn,len);
        case 'pchip'
            data(st:en) = interpolateNaNRangePchip(pp,p,n,nn,len);
        case 'linear'
            data(st:en) = interpolateNaNRangeLinear(pp,p,n,nn,len);
        otherwise
            error('Unsupport algorithm: %s',algorithm);
    end
end

if shifted ~= 0
    data = circshift(data,-shifted); % shift data back
end
end

%local functions
function data = interpolateNaNRangeDefault(pp,p,n,nn,numPoints,bounds)
if nargin < 6 || isempty(bounds)
    minVal = -inf;
    maxVal = inf;
else
    minVal = min(bounds);
    maxVal = max(bounds);
end

datavalid = @(data)min(data) > minVal && max(data) < maxVal;

data = interpolateNaNRangeSpline(pp,p,n,nn,numPoints);
if datavalid(data)
    return
end

data = interpolateNaNRangePchip(pp,p,n,nn,numPoints);
if datavalid(data)
    return
end

data = interpolateNaNRangeLinear(pp,p,n,nn,numPoints);
end

function data = interpolateNaNRangeSpline(pp,p,n,nn,numPoints)
data = spline([0 numPoints+1],[p-pp,p,n,nn-n],1:numPoints)';
end

function data = interpolateNaNRangePchip(pp,p,n,nn,numPoints)
data = pchip([-1 0 numPoints+1 numPoints+2],[pp,p,n,nn],1:numPoints)';
end

function data = interpolateNaNRangeLinear(pp,p,n,nn,numPoints)
data = linspace(p,n,numPoints + 2)';
data([1,end]) = [];
end


%--------------------------------------------------------------------------%
% interpolateCircularNaNRanges.m                                           %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
