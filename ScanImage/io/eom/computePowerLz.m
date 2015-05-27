function Lz= computePowerLz(z,P)
%COMPUTEPOWERLZ Computes length constant Lz from (z,P) data
%
%% NOTES
%Returns a scalar Lz representing best 'fit' value for supplied data. 
%If an error occurs, an empty array is returned.
%
%% CHANGES
%   VI011110A: Only handle case of 2 points for now, to avoid using Stats toolbox. Not using more than 2 points anywhere at moment. -- Vijay Iyer 1/11/10
%
%% ***********************************

global state

if length(z) ~= length(P)
    error('Power and depth arrays must have equal length');    
else
    %Sort z into ascending order
    [z,ix] = sort(z);
    P = P(ix);
    
    %Obtain all pairwise combinations
    %zPairs = combnk(z,2); %VI011110A
    %PPairs = combnk(P,2); %VI011110A
    zPairs = z; %VI011110A
    PPairs = P; %VI011110A
    
    %Compute Lz for each pair
    Lz = zeros(size(zPairs,1),1);
    for i=1:size(zPairs,1)
        Lz(i) = computePairwise(z(i,:),P(i,:));
    end
    
    %Test for obvious red flags
    if ~(all(sign(Lz) >= 0) || all(sign(Lz) <= 0)) %Lz values should be all positive or negative
        Lz = [];
    else
        %Return the mean value as the best fit
        Lz = mean(Lz);
    end
    
    
end

    %Each array must be of length 2
    function Lz = computePairwise(z,P)
        Lz = -1^(state.motor.zDepthPositive)*-1*(z(2)-z(1))/(log(P(2)/P(1)));       
    end

    
end


