% @daqmanager/deleteChannels - Remove any channels that may exist on the daq objects.
%
%% SYNTAX
%  deleteChannels(DAQMANAGER, daqobjects)
%   daqobjects - Matlab analoginput and/or analogoutput objects.
%
%% NOTES
%
%% CHANGES
%       VI080508A Vijay Iyer 8/5/08 - Use deleteAOChannels to deal with DAQ toolbox bug affecting sample rate property
%
%% CREDITS
% Created 4/7/05 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005    
%% *************************************************************
function deleteChannels(this, daqobjects)

if strcmpi(class(daqobjects), 'cell')
    for i = 1 : length(daqobjects)
        try
            if ~isempty(daqobjects(i))
                deletionArray = [];

                if ~isempty(daqobjects{i})
                    for j = 1 : length(daqobjects{i}.Channel)
                        deletionArray(length(deletionArray) + 1) = j;
                    end
                end

                if ~isempty(deletionArray)
                    if isa(daqobjects{i},'analogoutput') %VI080508A
                        deleteAOChannels(daqobjects{i},deletionArray); %VI080508A
                    else
                        delete(daqobjects{i}.Channel(deletionArray));
                    end
                end
            end
        catch
            warning('Skipping delete of channels on board #%s. Error: %s', num2str(i), lasterr);
        end
    end

else
    for i = 1 : length(daqobjects)
        try
            if ~isempty(daqobjects(i))
                deletionArray = [];
                
                if ~isempty(daqobjects(i))
                    for j = 1 : length(daqobjects(i).Channel)
                        deletionArray(length(deletionArray) + 1) = j;
                    end
                end

                if ~isempty(deletionArray)
                    if isa(daqobjects{i},'analogoutput') %VI080508A
                        deleteAOChannels(daqobjects{i},deletionArray); %VI080508A
                    else
                        delete(daqobjects(i).Channel(deletionArray));
                    end
                    %                 else %TO022805a
                    %                     warning('Failed to locate output channel(s) for deletion in set {%s}', varargin{:});
                end
            end
        catch
            warning('Skipping delete of channels on board #%s. Error: %s', num2str(i), lasterr);
        end
    end
end

return;