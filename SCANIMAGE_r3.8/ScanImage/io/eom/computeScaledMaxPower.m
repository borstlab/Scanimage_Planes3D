function scaledMaxPower = computeScaledMaxPower(beam)
%COMPUTESCALEDMAXPOWER Computes scaled max power for specified beam
global state

%scaledMaxPower = max(min(round(state.init.eom.maxPower(beam) * state.init.eom.stackPowerScaling(beam)),state.init.eom.maxLimit(beam)),1); %VI011210A
scaledMaxPower = max(min(state.init.eom.maxPower(beam) * state.init.eom.stackPowerScaling(beam),state.init.eom.maxLimit(beam)),1);  %VI011210A

end

