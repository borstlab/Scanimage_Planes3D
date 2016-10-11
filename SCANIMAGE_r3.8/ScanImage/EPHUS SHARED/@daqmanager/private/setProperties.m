% @daqmanager/setProperties - Set the channel properties on the correct analoginput/analogoutput objects.
%
% SYNTAX
%  setProperties(this, name, ...)
%  setProperties(this, nameArray)
%   name - A valid channel name.
%   nameArray - A cell array of valid channel names.
%
% USAGE
%  Set daqobject properties from a list of channel names (the list may be a cell array).
%
% NOTES
%  Moved from a subfunction in startChannel to a private function in the flass.
%
% CHANGES
%  TO080606A: See @daqmanager/startChannel for changes prior to refactoring. -- Tim O'Connor 8/6/06
%
% Created 8/4/06 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2006
function setProperties(dm, varargin)
global gdm;

if length(varargin) == 1 & strcmpi(class(varargin{1}), 'cell')
    varargin = varargin{1};
end

done = {};

%Use the properties associated with any channel, since, by definition
%there should be no conflicts.
for i = 1 : length(varargin)
    index = getChannelIndex(dm, varargin{i});
    
    %Look for an output.
    if gdm(dm.ptr).channels(index).ioFlag == 0
        obj = getAO(dm, gdm(dm.ptr).channels(index).boardId);
    elseif gdm(dm.ptr).channels(index).ioFlag == 1
        obj = getAI(dm, gdm(dm.ptr).channels(index).boardId);
    end
    
    %If we've already done this board's object, or it doesn't exist, or has no channels skip to the next.
    if isempty(obj)
        continue;
        %Tim O'Connor 9/28/04 - Can't reference fields in an empty struct? TO092804b
    elseif isempty(obj.Channel)
        continue;
    elseif inList(obj, done)
        continue;
    end
    
%     index = getChannelIndex(dm, varargin{i});
    
    if gdm(dm.ptr).channels(index).ioFlag == 0
        %TO022706D: Optimization(s). Use flags to determine which properties actually need to be modified on the board. -- Tim O'Connor 2/27/06
%         parameters = interleaveArrays({gdm(dm.ptr).channels(index).aoProps{:, 1}}, {gdm(dm.ptr).channels(index).aoProps{:, 2}});
        parameterIndices = find(gdm(dm.ptr).channels(index).aoPropsModificationFlags);
        if ~isempty(parameterIndices)
            parameters = interleaveArrays({gdm(dm.ptr).channels(index).aoProps{parameterIndices, 1}}, ...
                {gdm(dm.ptr).channels(index).aoProps{parameterIndices, 2}});
            set(obj, parameters{:});
            gdm(dm.ptr).channels(index).aoPropsModificationFlags(:) = 0;%TO022706D
        end
%         for j = 1 : size(gdm(dm.ptr).channels(index).aoProps, 1)
%             %Setup output properties.
%             name = gdm(dm.ptr).channels(index).aoProps{j, 1};
% 
%             %The Matlab 6.5 default for BufferingConfig is unusable. -- TO041305A Tim O'Connor 4/13/05
%             if ~(strcmpi(name, 'BufferingConfig') & isempty(gdm(dm.ptr).channels(index).aoProps{j, 2}))
% 
%                 set(obj, name, gdm(dm.ptr).channels(index).aoProps{j, 2});
%             end
%         end
    elseif gdm(dm.ptr).channels(index).ioFlag == 1
        %TO022706D: Optimization(s). Use flags to determine which properties actually need to be modified on the board. -- Tim O'Connor 2/27/06
%         parameters = interleaveArrays({gdm(dm.ptr).channels(index).aiProps{:, 1}}, {gdm(dm.ptr).channels(index).aiProps{:, 2}});
        parameterIndices = find(gdm(dm.ptr).channels(index).aiPropsModificationFlags);
        if ~isempty(parameterIndices)
            parameters = interleaveArrays({gdm(dm.ptr).channels(index).aiProps{parameterIndices, 1}}, ...
                {gdm(dm.ptr).channels(index).aiProps{parameterIndices, 2}});
            set(obj, parameters{:});
            gdm(dm.ptr).channels(index).aiPropsModificationFlags(:) = 0;%TO022706D
        end
%         for j = 1 : size(gdm(dm.ptr).channels(index).aiProps, 1)
%             %Setup input properties.
%             name = gdm(dm.ptr).channels(index).aiProps{j, 1};
% 
%             %The Matlab 6.5 default for BufferingConfig is unusable. -- TO041305A Tim O'Connor 4/13/05
%             if ~(strcmpi(name, 'BufferingConfig') & isempty(gdm(dm.ptr).channels(index).aiProps{j, 2})) ...
%                 & ~strcmpi(name, 'TriggerCondition') & ~strcmpi(name, 'ChannelSkew')
% % if strcmpi(gdm(dm.ptr).channels(index).aiProps{i, 1}, 'SamplesAcquiredFcn') & strcmpi(varargin{i}, 'AXOPATCH_200B_1_scaledOutput')
% % %     dm.ptr
% % %     index
% % %     i   
% % %     gdm(dm.ptr).channels(index).name
% % %     gdm(dm.ptr).channels(index).aiProps{i, 2}
% % fprintf(1, 'gdm(%s).channels(%s).aiProps{%s, :}\n', num2str(dm.ptr), num2str(index), num2str(j));
% % end
% fprintf(1, 'Setting properties - %s:''%s''\n', get(obj, 'Name'), name);
% gdm(dm.ptr).channels(index).aiProps{j, 2}
%                 set(obj, name, gdm(dm.ptr).channels(index).aiProps{j, 2});
%             end
%         end
    end
    
    %TO032406F - Moved TO0112205C into startAIs and startAOs. -- Tim O'Connor 3/24/06
    
    done{length(done) + 1} = obj;
end

return;