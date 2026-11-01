function initialize_generator!(inputs::Inputs, generator::ClassicGenerator, state::ClassicGeneratorState)
    bus = get_bus(inputs, generator_bus(generator))
    V0 = bus_voltage(bus) * cis(bus_angle(bus))
    S0 = generator_P0(generator) + im * generator_Q0(generator)
    I0 = conj(S0 / V0)
    E = V0 + generator_impedance(inputs, generator) * I0

    state.Eq_prime = abs(E)
    state.delta[1] = angle(E)
    state.omega[1] = 1.0

    return nothing
end

function initialize_generator!(inputs::Inputs, generator::OneDOneQGenerator, state::OneDOneQGeneratorState)
    # TODO: The power flow is read from the case data with a limited number of digits, so the network solved at
    # t = 0 does not reproduce V0 and I0 exactly. As a result, the derivatives of ed_prime and eq_prime at t = 0
    # are about 1e-8 instead of zero, and the operating point drifts slowly without any perturbation.
    # Solving the power flow inside nsLFT, or reading it with more digits, would remove this drift
    bus = get_bus(inputs, generator_bus(generator))
    V0 = bus_voltage(bus) * cis(bus_angle(bus))
    S0 = generator_P0(generator) + im * generator_Q0(generator)
    I0 = conj(S0 / V0)

    Ra = generator_Ra(inputs, generator)
    Xd = generator_Xd(inputs, generator)
    Xd_prime = generator_Xd_prime(inputs, generator)
    Xq = generator_Xq(inputs, generator)
    Xq_prime = generator_Xq_prime(inputs, generator)

    # in steady state the voltage behind Ra + jXq lies on the q axis, so its angle is the rotor angle
    E = V0 + (Ra + im * Xq) * I0

    state.delta[1] = angle(E)
    state.omega[1] = 1.0

    T = ri_to_dq(state.delta[1])
    vd, vq = T * [real(V0); imag(V0)]
    state.id[1], state.iq[1] = T * [real(I0); imag(I0)]
    state.ed_prime[1] = vd + Ra * state.id[1] - Xq_prime * state.iq[1]
    state.eq_prime[1] = vq + Ra * state.iq[1] + Xd_prime * state.id[1]
    state.efd[1] = state.eq_prime[1] + (Xd - Xd_prime) * state.id[1]

    return nothing
end

function initialize_generators!(inputs::Inputs, results::Results)
    for (generator, state) in zip(inputs.generators, results.generator)
        initialize_generator!(inputs, generator, state)
    end
    return nothing
end
