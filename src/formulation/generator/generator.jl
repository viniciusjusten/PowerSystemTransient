# The internal EMF depends on the machine model, and the governor and the voltage regulator dispatch on their own types,
# so these functions are the same for every generator
function predict_generator!(inputs::Inputs, generator::Generator, state::GeneratorState, ts::Int)
    governor = generator_governor(generator)
    voltage_regulator = generator_voltage_regulator(generator)
    predict_omega!(inputs, generator, state, ts)
    predict_delta!(inputs, state, ts)
    predict_Pmec_1!(inputs, generator, governor, state.governor, state, ts)
    predict_Pmec!(inputs, governor, state.governor, state, ts)
    predict_internal_emf!(inputs, generator, state, ts)
    predict_efd!(inputs, voltage_regulator, state.voltage_regulator, state, ts)
    return nothing
end

# each state takes the values at ts + 1 already updated in the same iteration, so the order of the calls matters
function step_generator!(inputs::Inputs, generator::Generator, state::GeneratorState, ts::Int)
    governor = generator_governor(generator)
    voltage_regulator = generator_voltage_regulator(generator)
    step_omega!(inputs, generator, state, ts)
    step_delta!(inputs, state, ts)
    step_Pmec_1!(inputs, generator, governor, state.governor, state, ts)
    step_Pmec!(inputs, governor, state.governor, state, ts)
    step_internal_emf!(inputs, generator, state, ts)
    step_efd!(inputs, voltage_regulator, state.voltage_regulator, state, ts)
    return nothing
end

# In the dynamic phasor network the stator keeps the derivative of its flux. With Xd' = Xq' = X', the stator flux in the
# network frame is psi = -j E - X' I, and the Park equations V = (1 / wb) dpsi/dt + j psi - Ra I give
#   E - (j / wb) dE/dt - V = Z I + L dI/dt, with Z = Ra + j X' and L = X' / wb
# The stator is then the companion model of an RL, oriented from the internal EMF to the bus, so its voltage is E - V
# and its current is the generator current in the network frame, plus the transformer voltage of the rotor flux,
# -(j / wb) dE/dt. Since Xd' = Xq', the same functions serve every generator model, and only the internal EMF depends on
# the model

# the history current of the stator for the step from ts - 1 to ts, from its voltage and current at ts - 1
function generator_stator_history_current(
    network::NetworkDP,
    inputs::Inputs,
    results::Results,
    generator::Generator,
    state::GeneratorState,
    ts::Int,
)
    stator = generator_stator(inputs, generator)
    bus_state = results.bus[generator_bus(generator)]
    V_previous = bus_voltage_phasor(network, bus_state, ts - 1)
    E_previous = generator_internal_emf(generator, state, ts - 1)
    Ir, Ii = dq_to_ri(state.delta[ts-1]) * [state.id[ts-1]; state.iq[ts-1]]
    I_previous = Ir + im * Ii
    return companion_history_current(network, inputs, stator, E_previous - V_previous, I_previous)
end

# The transformer voltage of the rotor flux, -(j / wb) dE/dt, integrated over the step by the trapezoidal rule, adds
# -j kappa G (E(ts) - E(ts - 1)) to the stator current, with kappa = 2 / (wb h). It is zero in steady state, where E is
# constant in the network frame, and when the rotor speed differs from 1 pu the rotation of E makes it the speed
# voltage (omega - 1) E
function generator_stator_flux_current(
    network::NetworkDP,
    inputs::Inputs,
    generator::Generator,
    state::GeneratorState,
    ts::Int,
)
    stator = generator_stator(inputs, generator)
    G = companion_conductance(network, inputs, stator)
    h = integration_step(inputs)
    wb = 2 * pi * fb(inputs)
    kappa = 2 / (wb * h)
    E = generator_internal_emf(generator, state, ts)
    E_previous = generator_internal_emf(generator, state, ts - 1)
    return -im * kappa * G * (E - E_previous)
end

function generator_current_dq(
    network::NetworkDP,
    inputs::Inputs,
    results::Results,
    generator::Generator,
    state::GeneratorState,
    ts::Int,
)
    if generator_tripped(generator)
        return [0.0, 0.0]
    end
    stator = generator_stator(inputs, generator)
    G = companion_conductance(network, inputs, stator)
    I_hist = generator_stator_history_current(network, inputs, results, generator, state, ts)
    I_flux = generator_stator_flux_current(network, inputs, generator, state, ts)
    bus_state = results.bus[generator_bus(generator)]
    V = bus_voltage_phasor(network, bus_state, ts)
    E = generator_internal_emf(generator, state, ts)
    I = G * (E - V) + I_hist + I_flux
    Idq = ri_to_dq(state.delta[ts]) * [real(I); imag(I)]
    return Idq
end

# Norton equivalent of the stator. Its conductance G is in the companion matrix, so the injection is the part of the
# stator current that does not depend on V, G E + I_hist + I_flux
function generator_current_network_contribution(
    network::NetworkDP,
    inputs::Inputs,
    results::Results,
    generator::Generator,
    state::GeneratorState,
    ts::Int,
)
    stator = generator_stator(inputs, generator)
    G = companion_conductance(network, inputs, stator)
    I_hist = generator_stator_history_current(network, inputs, results, generator, state, ts)
    I_flux = generator_stator_flux_current(network, inputs, generator, state, ts)
    E = generator_internal_emf(generator, state, ts)
    return G * E + I_hist + I_flux
end

# terminal voltage of the generator is the voltage at the bus to which it is connected
function terminal_voltage(inputs::Inputs, results::Results, generator::Generator, ts::Int)
    network = get_network(inputs)
    bus_state = results.bus[generator_bus(generator)]
    V = bus_voltage_phasor(network, bus_state, ts)
    return abs(V)
end
