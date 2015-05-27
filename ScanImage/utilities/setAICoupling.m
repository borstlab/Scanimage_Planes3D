function setAICoupling(aiObjOrChan,couplingType)
%SETAICOUPLING Sets AI Coupling property in a fashion that's compatible for all boards and for all DAQ driver versions
%% SYNTAX
%   setAICoupling(aiObjOrChan, couplingType)
%       aiObjOrChan: An AI object or an AI object's channel array. If an AI object, /all/ of its channels are set accordingly.
%       couplingType: A string, either 'AC' or 'DC', indicating the type of coupling to use on the AI channel(s)
%% NOTES
%       Not all boards have the AI coupling featureproperty; this function does nothing if such a board is indicated
%   
%       Earlier versions of the DAQ toolbox mis-spelled the property as 'coupling' rather than 'Coupling'.
%% CREDITS
%   Created 8/14/08 by Vijay Iyer
%% *************************************************

if isa(aiObjOrChan,'analoginput')
    chanArray = get(aiObjOrChan,'Channel');
elseif isa(aiObjOrChan, 'aichannel')
    chanArray = aiObjOrChan;
else
    error('First argument must specify an analoginput or aichannel object');
end

if ~ischar(couplingType) || ~ismember(lower(couplingType),{'ac' 'dc'})
    error('Invalid couplingType parameter -- must be either ''AC'' or ''DC''');
end

chanProps = propinfo(chanArray);

if isfield(chanProps,'coupling')
    set(chanArray,'coupling', couplingType);
elseif isfield(chanProps,'Coupling')
    set(chanArray,'Coupling', couplingType);
end

