# internal EMF in the network frame. E'q lies on the q axis, whose angle in the network frame is delta
function generator_internal_emf(::ClassicGenerator, state::ClassicGeneratorState, ts::Int)
    return state.Eq_prime * cis(state.delta[ts])
end

# Current
function generator_current(
    network::NetworkRMS,
    inputs::Inputs,
    results::Results,
    generator::ClassicGenerator,
    state::ClassicGeneratorState,
    ts::Int,
)
    if generator_tripped(generator)
        return 0.0 + 0.0im
    end
    bus_state = results.bus[generator_bus(generator)]
    V = bus_voltage_phasor(network, bus_state, ts)
    E = generator_internal_emf(generator, state, ts)
    I = (E - V) * generator_admittance(inputs, generator)
    return I
end

function generator_current_dq(
    network::NetworkRMS,
    inputs::Inputs,
    results::Results,
    generator::ClassicGenerator,
    state::ClassicGeneratorState,
    ts::Int,
)
    if generator_tripped(generator)
        return [0.0, 0.0]
    end
    Iri = generator_current(network, inputs, results, generator, state, ts)
    Idq = ri_to_dq(state.delta[ts]) * [real(Iri); imag(Iri)]
    return Idq
end

# results is not used here, but the OneDOneQ method needs the bus voltage and both are called the same way. The network
# only selects the method
function generator_current_network_contribution(
    ::NetworkRMS,
    inputs::Inputs,
    ::Results,
    generator::ClassicGenerator,
    state::ClassicGeneratorState,
    ts::Int,
)
    E = generator_internal_emf(generator, state, ts)
    I = E * generator_admittance(inputs, generator)
    return I
end

# Electrical power
# The electrical power is the power of the internal EMF, written with the stator currents id and iq at ts, which are
# computed before it. E'q lies on the q axis, so only iq carries power. The terminal voltage does not enter, so the
# same expression holds for every network formulation
function electrical_power(::Inputs, generator::ClassicGenerator, state::ClassicGeneratorState, ts::Int)
    if generator_tripped(generator)
        return 0.0
    end
    return state.Eq_prime * state.iq[ts]
end

# the internal EMF of the classic generator is constant, so there is nothing to integrate
function predict_internal_emf!(::Inputs, ::ClassicGenerator, ::ClassicGeneratorState, ::Int)
    return nothing
end

function step_internal_emf!(::Inputs, ::ClassicGenerator, ::ClassicGeneratorState, ::Int)
    return nothing
end
