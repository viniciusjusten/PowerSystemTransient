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

# results is not used here, but the OneDOneQ method needs the bus voltage and both are called the same way
function generator_current_network_contribution(inputs::Inputs, ::Results, generator::ClassicGenerator, state::ClassicGeneratorState, ts::Int)
    E = state.Eq_prime * cis(state.delta[ts])
    I = E * generator_admittance(inputs, generator)
    return I
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

# the internal EMF of the classic generator is constant, so there is nothing to integrate
function predict_internal_emf!(::Inputs, ::ClassicGenerator, ::ClassicGeneratorState, ::Int)
    return nothing
end

function step_internal_emf!(::Inputs, ::ClassicGenerator, ::ClassicGeneratorState, ::Int)
    return nothing
end
