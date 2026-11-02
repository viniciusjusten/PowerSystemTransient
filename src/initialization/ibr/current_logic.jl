# The initial point is an equilibrium only if neither the voltage dip nor the current limit acts at t = 0. So Vt at
# t = 0 must not be below Vdip, the currents that deliver Pref and Qref at Vt must fit within Imax and, for an IBR that
# is not a battery, the active current must not be negative. Pref and Qref at t = 0 come from the active and reactive
# power controls, so they must be initialized before
function initialize_current_logic!(inputs::Inputs, ibr::IBR, state::IBRState)
    Vt = state.Vt[1]
    Vdip = ibr_Vdip(ibr)
    if Vt < Vdip
        error("Initial terminal voltage of the IBR $(ibr_name(ibr)) (Vt = $Vt) is below the voltage dip threshold (Vdip = $Vdip)")
    end

    update_current_logic!(inputs, ibr, state, 1)

    Ip = state.Ip_cmd_pre[1]
    Iq = state.Iq_cmd_pre[1]
    if Ip < 0.0 && !ibr_battery(ibr)
        error("Initial active current of the IBR $(ibr_name(ibr)) (Ip = $Ip) is negative, which only a battery allows")
    end
    current_magnitude = sqrt(Ip^2 + Iq^2)
    Imax = ibr_Imax(inputs, ibr)
    if current_magnitude > Imax
        error("Initial current of the IBR $(ibr_name(ibr)) (I = $current_magnitude) is above the converter current limit (Imax = $Imax)")
    end
    return nothing
end
