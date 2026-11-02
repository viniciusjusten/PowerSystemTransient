# Active power control with frequency droop and PI
function active_power_error(
    inputs::Inputs,
    ibr::IBR,
    state::IBRState,
    freq_estimation_state::BusFreqEstimationState,
    ts::Int,
)
    # the frequency is the one estimated at the bus of the IBR
    droop_power = ibr_D(inputs, ibr) * (1.0 - freq_estimation_state.omega[ts])
    return droop_power + state.Pset[ts] - state.Pmed[ts]
end

# xp and Pext are algebraic, so they are updated whenever the error or yp at ts change
function update_Pext!(
    inputs::Inputs,
    ibr::IBR,
    state::IBRState,
    freq_estimation_state::BusFreqEstimationState,
    ts::Int,
)
    ep = active_power_error(inputs, ibr, state, freq_estimation_state, ts)
    state.xp[ts] = ibr_Kpg(ibr) * ep
    unlimited_Pext = state.xp[ts] + state.yp[ts]
    state.Pext[ts] = clamp(unlimited_Pext, ibr_Pmin(inputs, ibr), ibr_Pmax(inputs, ibr))
    return nothing
end

# The integrator stops while the PI output is at a limit and the error pushes it further out (anti-windup), and during
# the voltage dip, Vt < Vdip, so it does not accumulate error while Pmed falls with a fault, which would cause a power
# spike after the clearing. The status is decided from the values at ts and kept for the whole step, at both ends of
# the trapezoidal rule. Deciding it at ts + 1 would depend on yp at ts + 1 itself, and when the limit is crossed within
# the step no value of yp at ts + 1 satisfies its own condition, so the corrector would alternate between two values
# without converging
function yp_frozen(
    inputs::Inputs,
    ibr::IBR,
    state::IBRState,
    freq_estimation_state::BusFreqEstimationState,
    ts::Int,
)
    if in_voltage_dip(ibr, state, ts)
        return true
    end
    ep = active_power_error(inputs, ibr, state, freq_estimation_state, ts)
    unlimited_Pext = state.xp[ts] + state.yp[ts]
    if unlimited_Pext >= ibr_Pmax(inputs, ibr) && ep > 0.0
        return true
    end
    if unlimited_Pext <= ibr_Pmin(inputs, ibr) && ep < 0.0
        return true
    end
    return false
end

function deriv_yp(
    inputs::Inputs,
    ibr::IBR,
    state::IBRState,
    freq_estimation_state::BusFreqEstimationState,
    ts::Int,
)
    ep = active_power_error(inputs, ibr, state, freq_estimation_state, ts)
    return ibr_Kig(ibr) * ep
end

function deriv_Pref(ibr::IBR, state::IBRState, ts::Int)
    return 1 / ibr_Tp(ibr) * (
        state.Pext[ts] - state.Pref[ts]
    )
end

function predict_active_power_control!(
    inputs::Inputs,
    ibr::IBR,
    state::IBRState,
    freq_estimation_state::BusFreqEstimationState,
    ts::Int,
)
    # a perturbation at ts solves the network again and changes Pmed at ts, so xp and Pext at ts are updated before
    # the derivatives at ts. xp and Pext at ts + 1 depend on Pmed at ts + 1, which only the network solution gives,
    # so they are computed in the step
    update_Pext!(inputs, ibr, state, freq_estimation_state, ts)

    # predict the next yp and Pref values using the Euler method
    if yp_frozen(inputs, ibr, state, freq_estimation_state, ts)
        state.yp[ts+1] = state.yp[ts]
    else
        state.yp[ts+1] =
            state.yp[ts] +
            integration_step(inputs) * deriv_yp(inputs, ibr, state, freq_estimation_state, ts)
    end

    state.Pref[ts+1] =
        state.Pref[ts] +
        integration_step(inputs) * deriv_Pref(ibr, state, ts)

    return nothing
end

function step_active_power_control!(
    inputs::Inputs,
    ibr::IBR,
    state::IBRState,
    freq_estimation_state::BusFreqEstimationState,
    ts::Int,
)
    # compute the next yp value using the trapezoidal rule
    if yp_frozen(inputs, ibr, state, freq_estimation_state, ts)
        state.yp[ts+1] = state.yp[ts]
    else
        state.yp[ts+1] =
            state.yp[ts] +
            integration_step(inputs) / 2 * (
                deriv_yp(inputs, ibr, state, freq_estimation_state, ts) +
                deriv_yp(inputs, ibr, state, freq_estimation_state, ts+1)
            )
    end

    # Pext at ts + 1 is updated with the new yp before the derivative of Pref at ts + 1
    update_Pext!(inputs, ibr, state, freq_estimation_state, ts+1)

    # compute the next Pref value using the trapezoidal rule
    state.Pref[ts+1] =
        state.Pref[ts] +
        integration_step(inputs) / 2 * (
            deriv_Pref(ibr, state, ts) +
            deriv_Pref(ibr, state, ts+1)
        )

    return nothing
end
