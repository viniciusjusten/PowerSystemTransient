mutable struct Results
    generator::Vector{GeneratorState}
    bus::Vector{BusState}
    ibr::Vector{IBRState}
end

function Results(inputs::Inputs)
    num_steps = num_time_steps(inputs) + 1

    generator_states = GeneratorState[allocate_generator_state(gen, num_steps) for gen in inputs.generators]
    bus_states = BusState[allocate_bus_state(bus, num_steps) for bus in inputs.buses]
    ibr_states = IBRState[IBRState(num_steps) for _ in inputs.ibrs]

    return Results(generator_states, bus_states, ibr_states)
end

num_generators(results::Results) = length(results.generator)

num_buses(results::Results) = length(results.bus)
