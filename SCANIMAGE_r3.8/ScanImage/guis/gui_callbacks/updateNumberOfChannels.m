function updateNumberOfChannels
global state

% updateNumberOfChannels.m******
% Function that calculates the total number of channels to acquire, image, save and focus.
%
% Written By: Thomas Pologruto
% Cold Spring Harbor Labs
% January 2, 2001

updateNumberOfChannelsSave;
updateNumberOfChannelsImage;
updateNumberOfChannelsAcquire;
updateNumberOfMax;