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

function predict_internal_emf!(inputs::Inputs, generator::OneDOneQGenerator, state::OneDOneQGeneratorState, ts::Int)
    predict_ed_prime!(inputs, generator, state, ts)
    predict_eq_prime!(inputs, generator, state, ts)
    return nothing
end

function step_internal_emf!(inputs::Inputs, generator::OneDOneQGenerator, state::OneDOneQGeneratorState, ts::Int)
    step_ed_prime!(inputs, generator, state, ts)
    step_eq_prime!(inputs, generator, state, ts)
    return nothing
end

# Current
function generator_current_dq(
    network::NetworkRMS,
    inputs::Inputs,
    results::Results,
    generator::OneDOneQGenerator,
    state::OneDOneQGeneratorState,
    ts::Int,
)
    if generator_tripped(generator)
        return [0.0, 0.0]
    end
    bus_state = results.bus[generator_bus(generator)]
    V = bus_voltage_phasor(network, bus_state, ts)
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

function generator_current(
    network::NetworkRMS,
    inputs::Inputs,
    results::Results,
    generator::OneDOneQGenerator,
    state::OneDOneQGeneratorState,
    ts::Int,
)
    Idq = generator_current_dq(network, inputs, results, generator, state, ts)
    Ir, Ii = dq_to_ri(state.delta[ts]) * Idq
    return Ir + im * Ii
end

function generator_current_network_contribution(
    network::NetworkRMS,
    inputs::Inputs,
    results::Results,
    generator::OneDOneQGenerator,
    state::OneDOneQGeneratorState,
    ts::Int,
)
    # Ybus includes the generator admittance, so its current times V is added back here. The terms in V cancel
    # when Xd' = Xq', and otherwise V comes from the previous network solution and converges with the corrector
    Iri = generator_current(network, inputs, results, generator, state, ts)
    bus_state = results.bus[generator_bus(generator)]
    V = bus_voltage_phasor(network, bus_state, ts)
    return Iri + V * generator_admittance(inputs, generator)
end

# Electrical power
# The electrical power is written with the internal EMF and the stator currents id and iq at ts, which are computed
# before it. With the stator equations vd = ed' - Ra id + Xq' iq and vq = eq' - Xd' id - Ra iq, the terminal power plus
# the losses in Ra, (vq + Ra iq) iq + (vd + Ra id) id, is equal to the expression below. It does not use the terminal
# voltage, so it also holds when the stator has the L di/dt term of the dynamic phasor network
function electrical_power(inputs::Inputs, generator::OneDOneQGenerator, state::OneDOneQGeneratorState, ts::Int)
    if generator_tripped(generator)
        return 0.0
    end
    Xd_prime = generator_Xd_prime(inputs, generator)
    Xq_prime = generator_Xq_prime(inputs, generator)
    id = state.id[ts]
    iq = state.iq[ts]
    Pele = state.ed_prime[ts] * id + state.eq_prime[ts] * iq + (Xq_prime - Xd_prime) * id * iq
    return Pele
end
