function val = si_getMinLinePeriodIncrement()
%SI_GETMINLINEPERIODINCREMENT Summary of this function goes here
%   Detailed explanation goes here

global state

if isempty(state.internal.minLinePeriodIncrement)
    state.internal.minLinePeriodIncrement = 1/gcd(state.internal.baseInputRate,state.internal.baseOutputRate); %VI021309A %Can increment/decrement line period by 100us and still have integer # of AI/AO samples
end

val = state.internal.minLinePeriodIncrement;

end

