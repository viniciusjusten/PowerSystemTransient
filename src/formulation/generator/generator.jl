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

# Magnetic equations for the OneDOneQ generator model
function deriv_ed_prime(inputs::Inputs, generator::OneDOneQGenerator, state::OneDOneQGeneratorState, ts::Int)
    return 1 / generator_Tq0_prime(generator) * (
        - state.ed_prime[ts] +
        (generator_Xq(inputs, generator) - generator_Xq_prime(inputs, generator)) * state.iq[ts]
    )
end

function deriv_eq_prime(inputs::Inputs, generator::OneDOneQGenerator, state::OneDOneQGeneratorState, ts::Int)
    return 1 / generator_Td0_prime(generator) * (
        - state.eq_prime[ts] -
        (generator_Xd(inputs, generator) - generator_Xd_prime(inputs, generator)) * state.id[ts] +
        state.efd[ts]
    )
end

function predict_ed_prime!(inputs::Inputs, generator::OneDOneQGenerator, state::OneDOneQGeneratorState, ts::Int)
    state.ed_prime[ts+1] = 
    state.ed_prime[ts] +
    integration_step(inputs) * deriv_ed_prime(inputs, generator, state, ts)
    return nothing
end

function predict_eq_prime!(inputs::Inputs, generator::OneDOneQGenerator, state::OneDOneQGeneratorState, ts::Int)
    state.eq_prime[ts+1] = 
    state.eq_prime[ts] +
    integration_step(inputs) * deriv_eq_prime(inputs, generator, state, ts)
    return nothing
end

function step_ed_prime!(inputs::Inputs, generator::OneDOneQGenerator, state::OneDOneQGeneratorState, ts::Int)
    state.ed_prime[ts+1] = 
    state.ed_prime[ts] +
    integration_step(inputs) / 2 * (
        deriv_ed_prime(inputs, generator, state, ts) +
        deriv_ed_prime(inputs, generator, state, ts+1)
    )
    return nothing
end

function step_eq_prime!(inputs::Inputs, generator::OneDOneQGenerator, state::OneDOneQGeneratorState, ts::Int)
    state.eq_prime[ts+1] = 
    state.eq_prime[ts] +
    integration_step(inputs) / 2 * (
        deriv_eq_prime(inputs, generator, state, ts) +
        deriv_eq_prime(inputs, generator, state, ts+1)
    )
    return nothing
end

# Current
function generator_current(inputs::Inputs, results::Results, generator::ClassicGenerator, state::ClassicGeneratorState, ts::Int)
    if generator_tripped(generator)
        return 0.0 + 0.0im
    end
    bus_state = results.bus[generator_bus(generator)]
    V = bus_state.Vr[ts] + im * bus_state.Vi[ts]
    E = state.Eq_prime * cis(state.delta[ts])
    I = (E - V) * generator_admittance(inputs, generator)
    return I
end

function generator_current_dq(inputs::Inputs, results::Results, generator::ClassicGenerator, state::ClassicGeneratorState, ts::Int)
    if generator_tripped(generator)
        return [0.0, 0.0]
    end
    Iri = generator_current(inputs, results, generator, state, ts)
    Idq = ri_to_dq(state.delta[ts]) * [real(Iri); imag(Iri)]
    return Idq
end

function generator_current_network_contribution(inputs::Inputs, results::Results, generator::ClassicGenerator, state::ClassicGeneratorState, ts::Int)
    E = state.Eq_prime * cis(state.delta[ts])
    I = E * generator_admittance(inputs, generator)
    return I
end

function generator_current_dq(inputs::Inputs, results::Results, generator::OneDOneQGenerator, state::OneDOneQGeneratorState, ts::Int)
    if generator_tripped(generator)
        return [0.0, 0.0]
    end
    bus_state = results.bus[generator_bus(generator)]
    V = bus_state.Vr[ts] + im * bus_state.Vi[ts]
    Ra = generator_Ra(inputs, generator)
    Xq_prime = generator_Xq_prime(inputs, generator)
    Xd_prime = generator_Xd_prime(inputs, generator)
    vd, vq = ri_to_dq(state.delta[ts]) * [real(V); imag(V)]
    ed_prime = state.ed_prime[ts]
    eq_prime = state.eq_prime[ts]
    A = [Ra -Xq_prime; Xd_prime Ra]
    Idq = A \ [ed_prime - vd; eq_prime - vq]
    return Idq
end

function generator_current(inputs::Inputs, results::Results, generator::OneDOneQGenerator, state::OneDOneQGeneratorState, ts::Int)
    Idq = generator_current_dq(inputs, results, generator, state, ts)
    Ir, Ii = dq_to_ri(state.delta[ts]) * Idq
    return Ir + im * Ii
end

function generator_current_network_contribution(inputs::Inputs, results::Results, generator::OneDOneQGenerator, state::OneDOneQGeneratorState, ts::Int)
    # Ybus includes the generator admittance, so its current times V is added back here. The terms in V cancel
    # when Xd' = Xq', and otherwise V comes from the previous network solution and converges with the corrector
    Iri = generator_current(inputs, results, generator, state, ts)
    bus_state = results.bus[generator_bus(generator)]
    V = bus_state.Vr[ts] + im * bus_state.Vi[ts]
    return Iri + V * generator_admittance(inputs, generator)
end

# Electrical power
function electrical_power(
    inputs::Inputs,
    results::Results,
    generator::ClassicGenerator,
    state::ClassicGeneratorState,
    ts::Int,
)
    if generator_tripped(generator)
        return 0.0
    end
    bus_state = results.bus[generator_bus(generator)]
    V = bus_state.Vr[ts] + im * bus_state.Vi[ts]
    E = state.Eq_prime * cis(state.delta[ts])
    I = (E - V) * generator_admittance(inputs, generator)
    return real(conj(I) * E)
end

function electrical_power(
    inputs::Inputs,
    results::Results,
    generator::OneDOneQGenerator,
    state::OneDOneQGeneratorState,
    ts::Int,
)
    if generator_tripped(generator)
        return 0.0
    end
    bus_state = results.bus[generator_bus(generator)]
    V = bus_state.Vr[ts] + im * bus_state.Vi[ts]
    Ra = generator_Ra(inputs, generator)
    vd, vq = ri_to_dq(state.delta[ts]) * [real(V); imag(V)]
    Pele = (vq + Ra * state.iq[ts]) * state.iq[ts] + (vd + Ra * state.id[ts]) * state.id[ts]
    return Pele
end

# terminal voltage of the generator is the voltage at the bus to which it is connected
function terminal_voltage(inputs::Inputs, generator::Generator, results::Results, ts::Int)
    bus_idx = generator_bus(generator)
    Vr = results.bus[bus_idx].Vr[ts]
    Vi = results.bus[bus_idx].Vi[ts]
    return sqrt(Vr^2 + Vi^2)
end
