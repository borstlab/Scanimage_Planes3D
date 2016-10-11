%@daqmanager/private/deleteAOChannels -- Deletes AO channels in a 'safe' way
%% SYNTAX
%   deleteAOChannels(ao,chanIndices)
%       ao: an AO object
%       chanIndices: indices of channels belonging to AO object, set for deletion
%% NOTES
%   This function is intended to address DAQ Toolbox bug (service request 1-6OTRR3) wherein deleting the last channel of an AO object causes the maximum sample rate constraint value to be corrupted
%% CREDITS
%   Created 8/5/08 by Vijay Iyer
%% ***********************************************
function deleteAOChannels(ao,chanIndices)

%Determine if we are going to delete the final channel    
if length(ao.Channel) <= length(chanIndices) 
    lastChannelNumbered = true;
else
    lastChannelNumbered = false;
end

if lastChannelNumbered    
    sampRateInfo = propinfo(ao,'SampleRate');
    if strcmpi(sampRateInfo.Constraint,'bounded')
        sampRate = get(ao,'SampleRate');
        set(ao,'SampleRate',max(sampRateInfo.ConstraintValue));
    end
end

delete(ao.Channel(chanIndices));

if lastChannelNumbered
    set(ao,'SampleRate',sampRate);
end




