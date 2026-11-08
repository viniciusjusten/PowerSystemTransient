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

# In the dynamic phasor network the stator is the companion model of an RL, oriented from the internal EMF to the bus,
# so its voltage is E - V and its current is the generator current in the network frame. Since Xd' = Xq', the same
# functions serve every generator model, and only the internal EMF depends on the model

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
    bus_state = results.bus[generator_bus(generator)]
    V = bus_voltage_phasor(network, bus_state, ts)
    E = generator_internal_emf(generator, state, ts)
    I = G * (E - V) + I_hist
    Idq = ri_to_dq(state.delta[ts]) * [real(I); imag(I)]
    return Idq
end

# Norton equivalent of the stator. Its conductance G is in the companion matrix, so the injection is G E + I_hist
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
    E = generator_internal_emf(generator, state, ts)
    return G * E + I_hist
end

# terminal voltage of the generator is the voltage at the bus to which it is connected
function terminal_voltage(inputs::Inputs, results::Results, generator::Generator, ts::Int)
    network = get_network(inputs)
    bus_state = results.bus[generator_bus(generator)]
    V = bus_voltage_phasor(network, bus_state, ts)
    return abs(V)
end
