function rounded = roundTo(value, precision)
% ROUNDTO - Rounds to the specified number of decimal places.
%
% SYNTAX
%     rounded = roundTo(myValue, myPrecision)
%
% ARGUMENTS
%     value - The value to get rounded.
%     precision - The number of decimal places of precision.
%
% RETURNS
%     rounded - The input value, to <precision> decimal places of accuracy.
%
% EXAMPLES
%     roundTo(1.2345, 3)
%     ans =
%           1.234
%
% SEE ALSO ROUND, FLOOR, CEIL, FIX
%
% CREATED
%     Timothy O'Connor 6/8/04
%     Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2004
if precision < 0
    error('MATLAB:badopt', 'Can not round to less than 0 decimal places or precision.');
end

magnitude = 10^precision;

rounded = round(magnitude * value) / magnitude;

return;