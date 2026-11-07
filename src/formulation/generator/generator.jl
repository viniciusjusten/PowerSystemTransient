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

# terminal voltage of the generator is the voltage at the bus to which it is connected
function terminal_voltage(inputs::Inputs, results::Results, generator::Generator, ts::Int)
    network = get_network(inputs)
    bus_state = results.bus[generator_bus(generator)]
    V = bus_voltage_phasor(network, bus_state, ts)
    return abs(V)
end
