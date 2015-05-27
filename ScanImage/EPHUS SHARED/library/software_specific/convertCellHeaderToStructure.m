function out=convertCellHeaderToStructure(in)
% CONVERTCELLHEADERTOSTRUCTURE   - Converts a Nx2 Cell array into a structure.
%   ICONVERTCELLHEADERTOSTRUCTURE takes a Nx2 input cell array of parameter
%   values pairs and yields a structure using the 1st column of the input
%   as fieldnmaes.
%
%   See also CELL2STRUCT

out=cell2struct(in(:,2),in(:,1));