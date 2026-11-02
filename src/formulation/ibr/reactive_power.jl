# Reactive power control with voltage droop Kc and PI
function reactive_power_error(inputs::Inputs, ibr::IBR, state::IBRState, ts::Int)
    # the droop Kc lets the IBR share the reactive power with other equipment that regulates the voltage nearby
    droop_voltage = ibr_Kc(inputs, ibr) * state.Qmed[ts]
    return state.Vref[ts] - state.Vt[ts] - droop_voltage
end

# xq and Qext are algebraic, so they are updated whenever the error or yq at ts change
function update_Qext!(inputs::Inputs, ibr::IBR, state::IBRState, ts::Int)
    eq = reactive_power_error(inputs, ibr, state, ts)
    state.xq[ts] = ibr_Kp(inputs, ibr) * eq
    unlimited_Qext = state.xq[ts] + state.yq[ts]
    state.Qext[ts] = clamp(unlimited_Qext, ibr_Qmin(inputs, ibr), ibr_Qmax(inputs, ibr))
    return nothing
end

# The integrator stops while the PI output is at a limit and the error pushes it further out (anti-windup), and while
# the terminal voltage is below Vfrz, so a fault does not wind it up and leave a high Qref after the clearing. As in
# yp_frozen, the status is decided from the values at ts and kept for the whole step, at both ends of the trapezoidal
# rule, so the corrector does not alternate between two values when a condition changes within the step
function yq_frozen(inputs::Inputs, ibr::IBR, state::IBRState, ts::Int)
    if state.Vt[ts] < ibr_Vfrz(ibr)
        return true
    end
    eq = reactive_power_error(inputs, ibr, state, ts)
    unlimited_Qext = state.xq[ts] + state.yq[ts]
    if unlimited_Qext >= ibr_Qmax(inputs, ibr) && eq > 0.0
        return true
    end
    if unlimited_Qext <= ibr_Qmin(inputs, ibr) && eq < 0.0
        return true
    end
    return false
end

function deriv_yq(inputs::Inputs, ibr::IBR, state::IBRState, ts::Int)
    eq = reactive_power_error(inputs, ibr, state, ts)
    return ibr_Ki(inputs, ibr) * eq
end

function deriv_Qref(ibr::IBR, state::IBRState, ts::Int)
    return 1 / ibr_Tq(ibr) * (
        state.Qext[ts] - state.Qref[ts]
    )
end

function predict_reactive_power_control!(inputs::Inputs, ibr::IBR, state::IBRState, ts::Int)
    # a perturbation at ts solves the network again and changes Vt and Qmed at ts, so xq and Qext at ts are updated
    # before the derivatives at ts. xq and Qext at ts + 1 depend on Vt and Qmed at ts + 1, which only the network
    # solution gives, so they are computed in the step
    update_Qext!(inputs, ibr, state, ts)

    # predict the next yq and Qref values using the Euler method
    if yq_frozen(inputs, ibr, state, ts)
        state.yq[ts+1] = state.yq[ts]
    else
        state.yq[ts+1] =
            state.yq[ts] +
            integration_step(inputs) * deriv_yq(inputs, ibr, state, ts)
    end

    state.Qref[ts+1] =
        state.Qref[ts] +
        integration_step(inputs) * deriv_Qref(ibr, state, ts)

    return nothing
end

function step_reactive_power_control!(inputs::Inputs, ibr::IBR, state::IBRState, ts::Int)
    # compute the next yq value using the trapezoidal rule
    if yq_frozen(inputs, ibr, state, ts)
        state.yq[ts+1] = state.yq[ts]
    else
        state.yq[ts+1] =
            state.yq[ts] +
            integration_step(inputs) / 2 * (
                deriv_yq(inputs, ibr, state, ts) +
                deriv_yq(inputs, ibr, state, ts+1)
            )
    end

    # Qext at ts + 1 is updated with the new yq before the derivative of Qref at ts + 1
    update_Qext!(inputs, ibr, state, ts+1)

    # compute the next Qref value using the trapezoidal rule
    state.Qref[ts+1] =
        state.Qref[ts] +
        integration_step(inputs) / 2 * (
            deriv_Qref(ibr, state, ts) +
            deriv_Qref(ibr, state, ts+1)
        )

    return nothing
end
