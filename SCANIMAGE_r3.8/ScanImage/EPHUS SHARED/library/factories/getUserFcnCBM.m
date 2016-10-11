% getUserFcnCBM - Gets the callbackmanager for user functions.
%
% SYNTAX
%  cbm = getUserFcnCBM
%   cbm - An @callbackManager instance. All calls to this function return the same instance.
%
% USAGE
%
% NOTES
%
% CHANGES
%
% Created 12/5/05 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function cbm = getUserFcnCBM(varargin)
global getUserFcnCBMVariable;

if isempty(getUserFcnCBMVariable)
    getUserFcnCBMVariable = callbackmanager;
end

cbm = getUserFcnCBMVariable;

return;