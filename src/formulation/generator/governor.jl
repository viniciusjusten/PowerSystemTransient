# Droop1stOrder governor model
function deriv_Pmec_1(
    inputs::Inputs,
    generator::Generator,
    governor::Droop1stOrder,
    governor_state::Droop1stOrderState,
    state::GeneratorState,
    ts::Int,
)
    R = governor_R(inputs, generator, governor)
    omega_ref = governor_omega_ref(inputs, governor)
    return 1 / governor_Tc(governor) * (
        governor_state.Pref[ts] +
        (omega_ref - state.omega[ts]) / R -
        governor_state.Pmec_1[ts]
    )
end

function predict_Pmec_1!(
    inputs::Inputs,
    generator::Generator,
    governor::Droop1stOrder,
    governor_state::Droop1stOrderState,
    state::GeneratorState,
    ts::Int,
)
    # predict the next Pmec_1 value using the Euler method
    governor_state.Pmec_1[ts+1] =
        governor_state.Pmec_1[ts] +
        integration_step(inputs) * deriv_Pmec_1(inputs, generator, governor, governor_state, state, ts)
    return nothing
end

function step_Pmec_1!(
    inputs::Inputs,
    generator::Generator,
    governor::Droop1stOrder,
    governor_state::Droop1stOrderState,
    state::GeneratorState,
    ts::Int,
)
    # compute the next Pmec_1 value using the trapezoidal rule
    governor_state.Pmec_1[ts+1] =
        governor_state.Pmec_1[ts] +
        integration_step(inputs) / 2 * (
            deriv_Pmec_1(inputs, generator, governor, governor_state, state, ts) +
            deriv_Pmec_1(inputs, generator, governor, governor_state, state, ts+1)
        )
    return nothing
end

function deriv_Pmec(
    inputs::Inputs,
    generator::Generator,
    governor::Droop1stOrder,
    governor_state::Droop1stOrderState,
    state::GeneratorState,
    ts::Int,
)
    return 1 / governor_Tr(governor) * (
        governor_state.Pmec_1[ts] - state.Pmec[ts]
    )
end

function predict_Pmec!(
    inputs::Inputs,
    generator::Generator,
    governor::Droop1stOrder,
    governor_state::Droop1stOrderState,
    state::GeneratorState,
    ts::Int,
)
    # predict the next Pmec value using the Euler method
    state.Pmec[ts+1] =
        state.Pmec[ts] +
        integration_step(inputs) * deriv_Pmec(inputs, generator, governor, governor_state, state, ts)
    return nothing
end

function step_Pmec!(
    inputs::Inputs,
    generator::Generator,
    governor::Droop1stOrder,
    governor_state::Droop1stOrderState,
    state::GeneratorState,
    ts::Int,
)
    # compute the next Pmec value using the trapezoidal rule
    state.Pmec[ts+1] =
        state.Pmec[ts] +
        integration_step(inputs) / 2 * (
            deriv_Pmec(inputs, generator, governor, governor_state, state, ts) +
            deriv_Pmec(inputs, generator, governor, governor_state, state, ts+1)
        )
    return nothing
end

# without a governor there is no Pmec_1 and the mechanical power stays constant
function predict_Pmec_1!(::Inputs, ::Generator, ::Nothing, ::Nothing, ::GeneratorState, ::Int)
    return nothing
end

function step_Pmec_1!(::Inputs, ::Generator, ::Nothing, ::Nothing, ::GeneratorState, ::Int)
    return nothing
end

function predict_Pmec!(::Inputs, ::Generator, ::Nothing, ::Nothing, state::GeneratorState, ts::Int)
    state.Pmec[ts+1] = state.Pmec[ts]
    return nothing
end

function step_Pmec!(::Inputs, ::Generator, ::Nothing, ::Nothing, state::GeneratorState, ts::Int)
    state.Pmec[ts+1] = state.Pmec[ts]
    return nothing
end
