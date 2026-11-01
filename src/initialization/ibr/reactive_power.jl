# As for the active power, the setpoint comes from the network of nsLFT at t = 0 rather than from the power flow, so
# Vt and Qmed at t = 0 must come from the network solution. Vref is chosen so that the error starts at zero,
# Vref = Vt + Kc * Qmed, so xq = 0 and the integrator holds the whole output, yq = Qmed, which gives Qext = Qref = Qmed
function initialize_reactive_power_control!(inputs::Inputs, ibr::IBR, state::IBRState)
    Qmed = state.Qmed[1]
    Qmin = ibr_Qmin(inputs, ibr)
    Qmax = ibr_Qmax(inputs, ibr)
    if !(Qmed >= Qmin && Qmed <= Qmax)
        error("Initial reactive power of the IBR $(ibr_name(ibr)) (Qmed = $Qmed) is outside the limits of the reactive power control (Qmin = $Qmin, Qmax = $Qmax)")
    end
    Vref = state.Vt[1] + ibr_Kc(inputs, ibr) * Qmed
    state.Vref .= Vref
    state.xq[1] = 0.0
    state.yq[1] = Qmed
    state.Qext[1] = Qmed
    state.Qref[1] = Qmed
    return nothing
end
