# Converter simplified from WECC REGC_A. Ip and Iq follow the commands of the current logic with the lag
# 1 / (1 + s Tg), the rise of Ip is limited by Rrpwr and the low voltage gain reduces the active current that reaches
# the network

# Low voltage active current management. alpha is Vt applied on the line with slope 1 / (lv1 - lv0) that is zero at
# lv0 and one at lv1, kept at zero below lv0 and at one above lv1
function low_voltage_gain(ibr::IBR, Vt::Float64)
    lv0 = ibr_lv0(ibr)
    lv1 = ibr_lv1(ibr)
    if Vt <= lv0
        return 0.0
    end
    if Vt >= lv1
        return 1.0
    end
    return (Vt - lv0) / (lv1 - lv0)
end

# Ip_ef is algebraic, so it is updated whenever Ip or Vt at ts change
function update_Ip_ef!(ibr::IBR, state::IBRState, ts::Int)
    alpha = low_voltage_gain(ibr, state.Vt[ts])
    state.Ip_ef[ts] = alpha * state.Ip[ts]
    return nothing
end

# The IBR is a current source in the network. Ip_ef and Iq are in the reference frame of the terminal voltage, so the
# injection depends on Vt, through the low voltage gain, and on its angle theta. Both come from the voltage at ts already
# in the results, which is the previous network solution of the corrector, the voltage at ts - 1 copied by
# predict_voltages! in the first solution of a step, or the power flow voltage at t = 0, and converge with the iterations
function ibr_current_network_contribution(results::Results, ibr::IBR, state::IBRState, ts::Int)
    bus_idx = ibr_bus(ibr)
    bus_state = results.bus[bus_idx]
    V = bus_state.Vr[ts] + im * bus_state.Vi[ts]
    Vt = abs(V)
    alpha = low_voltage_gain(ibr, Vt)
    Ip_ef = alpha * state.Ip[ts]
    theta = angle(V)
    return (Ip_ef - im * state.Iq[ts]) * cis(theta)
end

# Vt, Ip_ef and the powers at the terminal from the network solution at ts
function update_ibr_measurements!(results::Results, ibr::IBR, state::IBRState, ts::Int)
    bus_idx = ibr_bus(ibr)
    bus_state = results.bus[bus_idx]
    V = bus_state.Vr[ts] + im * bus_state.Vi[ts]
    state.Vt[ts] = abs(V)
    update_Ip_ef!(ibr, state, ts)
    state.Pmed[ts] = state.Vt[ts] * state.Ip_ef[ts]
    state.Qmed[ts] = state.Vt[ts] * state.Iq[ts]
    return nothing
end

function deriv_Ip(ibr::IBR, state::IBRState, ts::Int)
    return 1 / ibr_Tg(ibr) * (
        state.Ip_cmd[ts] - state.Ip[ts]
    )
end

function deriv_Iq(ibr::IBR, state::IBRState, ts::Int)
    return 1 / ibr_Tg(ibr) * (
        state.Iq_cmd[ts] - state.Iq[ts]
    )
end

# The rise of Ip is limited by Rrpwr, which sets how fast the active power comes back after a fault. As in yp_frozen,
# the status is decided from the values at ts and kept for the whole step, so when the derivative at ts is above Rrpwr
# the derivative is Rrpwr at both ends of the trapezoidal rule
function Ip_ramp_limited(inputs::Inputs, ibr::IBR, state::IBRState, ts::Int)
    return deriv_Ip(ibr, state, ts) > ibr_Rrpwr(inputs, ibr)
end

function predict_converter!(inputs::Inputs, ibr::IBR, state::IBRState, ts::Int)
    # a perturbation at ts solves the network again and changes Vt at ts, so the current logic at ts is updated before
    # the derivatives at ts
    voltage_dip = in_voltage_dip(ibr, state, ts)
    update_current_logic!(inputs, ibr, state, ts, voltage_dip)

    # predict the next Ip and Iq values using the Euler method
    if Ip_ramp_limited(inputs, ibr, state, ts)
        state.Ip[ts+1] =
            state.Ip[ts] +
            integration_step(inputs) * ibr_Rrpwr(inputs, ibr)
    else
        state.Ip[ts+1] =
            state.Ip[ts] +
            integration_step(inputs) * deriv_Ip(ibr, state, ts)
    end

    state.Iq[ts+1] =
        state.Iq[ts] +
        integration_step(inputs) * deriv_Iq(ibr, state, ts)

    return nothing
end

function step_converter!(inputs::Inputs, ibr::IBR, state::IBRState, ts::Int)
    # The voltage dip status is decided at ts and kept for the whole step, so the current logic at ts + 1 uses it too.
    # iq_inj jumps from zero to Kqv * (V0 - Vdip) when Vt crosses Vdip, and deciding it at ts + 1 could make the
    # corrector alternate between two values when Vt is near Vdip. The current logic at ts + 1 takes Pref and Qref from
    # the step of the controls, which come before the converter, and Vt from the previous network solution
    voltage_dip = in_voltage_dip(ibr, state, ts)
    update_current_logic!(inputs, ibr, state, ts+1, voltage_dip)

    # compute the next Ip value using the trapezoidal rule, or the ramp Rrpwr when the rise is limited
    if Ip_ramp_limited(inputs, ibr, state, ts)
        state.Ip[ts+1] =
            state.Ip[ts] +
            integration_step(inputs) * ibr_Rrpwr(inputs, ibr)
    else
        state.Ip[ts+1] =
            state.Ip[ts] +
            integration_step(inputs) / 2 * (
                deriv_Ip(ibr, state, ts) +
                deriv_Ip(ibr, state, ts+1)
            )
    end

    # compute the next Iq value using the trapezoidal rule
    state.Iq[ts+1] =
        state.Iq[ts] +
        integration_step(inputs) / 2 * (
            deriv_Iq(ibr, state, ts) +
            deriv_Iq(ibr, state, ts+1)
        )

    return nothing
end
