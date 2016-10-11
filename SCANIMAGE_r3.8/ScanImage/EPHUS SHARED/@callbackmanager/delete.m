% @callbackManager/delete - An object used to track callbacks bound to strings, with unique identifiers.
%
%  SYNTAX
%   delete(cbm)
%    cbm - A @callbackmanager instance.
%
%  CHANGES
%
% Created 5/31/08 - Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2008
function delete(this)
global callbackmanagers;

callbackmanagers(this.ptr).callbacks = {};
callbackmanagers(this.ptr).handlesOnly = 0;
callbackmanagers(this.ptr).enable = 0;
callbackmanagers(this.ptr).readOnlyFields = {};

return;