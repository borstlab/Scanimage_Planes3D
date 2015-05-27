% --------------------------------------------------------------------
% Load the state data for Pockels cell operation.
function loadEomState(varargin);
global state;

if exist('PockelsCalibrationCurve.mat') ~= 2
    return;
end

load PockelsCalibrationCurve;

state.init.eom.lut = lut;
state.init.eom.min = eom_min;
state.init.eom.maxPower = maxPower;
state.init.eom.maxLimit = maxLimit;

clear lut eom_min maxPower maxLimit;