# Current logic simplified from WECC REEC_B. All the values are algebraic, functions of Vt at ts and of the states Pref
# and Qref at ts, so they are updated after the network solution at ts

# During the voltage dip, Vt < Vdip, the IBR injects a reactive current proportional to the voltage drop from the
# power flow solution, to support the voltage during the fault
function reactive_current_injection(inputs::Inputs, ibr::IBR, state::IBRState, ts::Int)
    if state.Vt[ts] < ibr_Vdip(ibr)
        bus_idx = ibr_bus(ibr)
        bus = get_bus(inputs, bus_idx)
        V0 = bus_voltage(bus)
        unlimited_iq_inj = ibr_Kqv(inputs, ibr) * (V0 - state.Vt[ts])
        return clamp(unlimited_iq_inj, ibr_Iql1(inputs, ibr), ibr_Iqh1(inputs, ibr))
    end
    return 0.0
end

# only a battery absorbs active power, so the lower limit of Ip_cmd is zero otherwise
function limit_Ip_cmd!(ibr::IBR, state::IBRState, ts::Int)
    Ip_min = 0.0
    if ibr_battery(ibr)
        Ip_min = -state.Ip_max[ts]
    end
    state.Ip_cmd[ts] = clamp(state.Ip_cmd_pre[ts], Ip_min, state.Ip_max[ts])
    return nothing
end

function limit_Iq_cmd!(state::IBRState, ts::Int)
    state.Iq_cmd[ts] = clamp(state.Iq_cmd_pre[ts], -state.Iq_max[ts], state.Iq_max[ts])
    return nothing
end

function update_current_logic!(inputs::Inputs, ibr::IBR, state::IBRState, ts::Int)
    state.iq_inj[ts] = reactive_current_injection(inputs, ibr, state, ts)

    # the floor on the voltage keeps a bolted fault at the terminal from dividing by zero
    Vt = max(state.Vt[ts], 1e-3)
    state.Ip_cmd_pre[ts] = state.Pref[ts] / Vt
    state.Iq_cmd_pre[ts] = state.iq_inj[ts] + state.Qref[ts] / Vt

    # The priority current is limited by Imax and the other one by what is left of Imax after the priority current
    # command, so the priority command is computed first. Since its magnitude is at most Imax, the square root is real
    Imax = ibr_Imax(inputs, ibr)
    if ibr_priority(ibr) == CurrentPriority.P
        state.Ip_max[ts] = Imax
        limit_Ip_cmd!(ibr, state, ts)
        state.Iq_max[ts] = sqrt(Imax^2 - state.Ip_cmd[ts]^2)
        limit_Iq_cmd!(state, ts)
    else
        state.Iq_max[ts] = Imax
        limit_Iq_cmd!(state, ts)
        state.Ip_max[ts] = sqrt(Imax^2 - state.Iq_cmd[ts]^2)
        limit_Ip_cmd!(ibr, state, ts)
    end
    return nothing
end
