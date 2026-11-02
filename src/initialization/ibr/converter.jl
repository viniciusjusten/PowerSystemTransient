# Before the network solution at t = 0, Ip and Iq come from the power flow, so the IBR injects P0 and Q0 at the power
# flow voltage, P0 = V0 * Ip and Q0 = V0 * Iq, with the low voltage gain equal to one
function initialize_converter_from_power_flow!(inputs::Inputs, ibr::IBR, state::IBRState)
    bus_idx = ibr_bus(ibr)
    bus = get_bus(inputs, bus_idx)
    V0 = bus_voltage(bus)
    state.Ip[1] = ibr_P0(ibr) / V0
    state.Iq[1] = ibr_Q0(ibr) / V0
    return nothing
end

# After the current logic at t = 0, Ip and Iq take the commands, so their derivatives start at zero. The commands
# deliver Pref = Pmed and Qref = Qmed at Vt, so they equal the currents injected at t = 0 but for the rounding. This is
# an equilibrium only if the low voltage gain is one at t = 0, since Ip_cmd = Pref / Vt does not account for it
function initialize_converter!(ibr::IBR, state::IBRState)
    Vt = state.Vt[1]
    lv1 = ibr_lv1(ibr)
    if Vt < lv1
        error("Initial terminal voltage of the IBR $(ibr_name(ibr)) (Vt = $Vt) is below lv1 = $lv1, so the low voltage gain of the active current would start below one")
    end
    state.Ip[1] = state.Ip_cmd[1]
    state.Iq[1] = state.Iq_cmd[1]
    update_Ip_ef!(ibr, state, 1)
    return nothing
end
