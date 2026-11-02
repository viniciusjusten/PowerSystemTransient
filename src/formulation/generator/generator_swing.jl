# Swing equation
function deriv_omega(inputs::Inputs, generator::Generator, state::GeneratorState, ts::Int)
    return 1 / (2 * generator_H(inputs, generator)) * (
        state.Pmec[ts] -
        state.Pele[ts] -
        generator_D(inputs, generator) * (
            state.omega[ts] - 1.0
        )
    )
end

function deriv_delta(inputs::Inputs, state::GeneratorState, ts::Int)
    return 2 * pi * fb(inputs) * (
        state.omega[ts] - 1.0
    )
end

function step_omega!(inputs::Inputs, generator::Generator, state::GeneratorState, ts::Int)
    # compute the next omega value using the trapezoidal rule
    state.omega[ts+1] =
        state.omega[ts] +
        integration_step(inputs) / 2 * (
            deriv_omega(inputs, generator, state, ts) +
            deriv_omega(inputs, generator, state, ts+1)
        )
    return nothing
end

function predict_omega!(inputs::Inputs, generator::Generator, state::GeneratorState, ts::Int)
    # predict the next omega value using the Euler method
    state.omega[ts+1] =
        state.omega[ts] +
        integration_step(inputs) * deriv_omega(inputs, generator, state, ts)
    return nothing
end

function predict_delta!(inputs::Inputs, state::GeneratorState, ts::Int)
    # predict the next delta value using the Euler method
    state.delta[ts+1] =
        state.delta[ts] +
        integration_step(inputs) * deriv_delta(inputs, state, ts)
    return nothing
end

function step_delta!(inputs::Inputs, state::GeneratorState, ts::Int)
    # compute the next delta value using the trapezoidal rule
    state.delta[ts+1] =
        state.delta[ts] +
        integration_step(inputs) / 2 * (
            deriv_delta(inputs, state, ts) +
            deriv_delta(inputs, state, ts+1)
        )
    return nothing
end
