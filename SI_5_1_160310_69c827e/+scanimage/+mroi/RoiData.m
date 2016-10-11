classdef RoiData < handle & matlab.mixin.Copyable
    % class defining image data for one roi at multiple z depths
    % ro get the scanfield for a specific z use obj.roi.get(z);
    properties
        hRoi;                          % handle to roi
        zs;                            % [numeric] array of zs
        channels;                      % [numeric] array of channelnumbers in imageData
        imageData;                     % cell of cell arrays of image data for
                                       %     channels(1st index) and zs (2nd index)
                                       %     image data is transposed
        stripePosition = {};           % cell array of 1x2 start and end line of the current stripe for each z. if empty, current stripe is full frame
        stripeFullFrameNumLines = [];  % stripeFullFrameNumLines indicates the number of lines in the full frame for each z
        
        transposed = true;
        frameNumberAcq;                % [numeric] frame number in current acquisition (taken from stripe data)
        frameNumberAcqMode;            % [numeric] frame number in current acquisition mode (taken from stripe data)
        frameTimestamp;                % [numeric] time stamp in current acquisition mode (taken from stripe data)
    end
    
    properties
        acqParamBuffer;                % buffer holding frequently used parameters to limit parameter recomputation
    end
    
    methods
        function obj = castImageData(obj,newType)
            for iterChannels = 1:length(obj.imageData)
                for iterZs = 1:length(obj.imageData{iterChannels})
                    obj.imageData{iterChannels}{iterZs} = cast(obj.imageData{iterChannels}{iterZs},newType);
                end
            end
        end
        
        function obj = multiplyData(obj,factor)
            for iterChannels = 1:length(obj.imageData)
                for iterZs = 1:length(obj.imageData{iterChannels})
                    obj.imageData{iterChannels}{iterZs} = obj.imageData{iterChannels}{iterZs} .* cast(factor,'like',obj.imageData{iterChannels}{iterZs});
                end
            end
        end
        
        function merge(obj,hRoiData)
            % merge hRoiData into obj
            if isempty(hRoiData) || isempty(hRoiData.imageData)
                return
            end
            
            for iterChannels = 1:length(obj.imageData)
                for iterZs = 1:length(obj.imageData{iterChannels})
                    pos = obj.stripePosition{iterZs};
                    if obj.transposed
                        obj.imageData{iterChannels}{iterZs}(:,1:(pos(1)-1)) = cast(hRoiData.imageData{iterChannels}{iterZs}(:,1:(pos(1)-1)),'like',obj.imageData{iterChannels}{iterZs});
                        obj.imageData{iterChannels}{iterZs}(:,(pos(2)+1):end) = cast(hRoiData.imageData{iterChannels}{iterZs}(:,(pos(2)+1):end),'like',obj.imageData{iterChannels}{iterZs});
                    else
                        obj.imageData{iterChannels}{iterZs}(1:(pos(1)-1),:) = cast(hRoiData.imageData{iterChannels}{iterZs}(1:(pos(1)-1),:),'like',obj.imageData{iterChannels}{iterZs});
                        obj.imageData{iterChannels}{iterZs}((pos(2)+1):end,:) = cast(hRoiData.imageData{iterChannels}{iterZs}((pos(2)+1):end,:),'like',obj.imageData{iterChannels}{iterZs});
                    end
                end
            end
        end
        
        function resetData(obj)
            % merge hRoiData into obj            
            for iterChannels = 1:length(obj.imageData)
                for iterZs = 1:length(obj.imageData{iterChannels})
                    pos = obj.stripePosition{iterZs};
                    if obj.transposed
                        obj.imageData{iterChannels}{iterZs}(:,1:(pos(1)-1)) = 0;
                        obj.imageData{iterChannels}{iterZs}(:,(pos(2)+1):end) = 0;
                    else
                        obj.imageData{iterChannels}{iterZs}(1:(pos(1)-1),:) = 0;
                        obj.imageData{iterChannels}{iterZs}((pos(2)+1):end,:) = 0;
                    end
                end
            end
        end
        
        function resetDataToZero(obj)
            % merge hRoiData into obj            
            for iterChannels = 1:length(obj.imageData)
                for iterZs = 1:length(obj.imageData{iterChannels})
                    dims = size(obj.imageData{iterChannels}{iterZs});
                    obj.imageData{iterChannels}{iterZs} = zeros(dims,'like',obj.imageData{iterChannels}{iterZs});
                end
            end
        end
    end
end


%--------------------------------------------------------------------------%
% RoiData.m                                                                %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
