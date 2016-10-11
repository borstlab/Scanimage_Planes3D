%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  This function is a subfunction of initGUIs that allows the data to be
%  passed in as a cell array as opposed to a text file.
%  opens and interprets and initialization file 
%
%
%% CHANGES
%   VI012109A: Establish the ArrayString/Array convention strictly by implementing the unpacking here -- Vijay Iyer 1/22/09
%   VI060710A: Comment out code that should be obsoleted and was preventing correct processing of empty bracket ([]) values in INI files -- Vijay Iyer 6/7/10
%   VI100110A: Support vectorial string cell arrays (including empty cell array degenerate case) -- Vijay Iyer 10/1/10
%   VI100410A: Allow vectorial string cell arrays to contain quote characters -- Vijay Iyer 10/4/10
%   DEQ20110221: This function's logic has been refactored/generalized to parseInitializationFile(), which it now calls through to.

%% CREDITS
%   Created - Thomas Pologruto 1/28/04
%   Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%% *******************************************
function initGUIsFromCellArray(file)
parseInitializationFile(file,'initGUIs');