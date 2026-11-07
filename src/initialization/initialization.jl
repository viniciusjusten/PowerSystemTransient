# the bus voltages at t = 0 are the phasors of the power flow solution
function initialize_buses!(::PhasorNetwork, inputs::Inputs, results::Results)
    for (bus, state) in zip(inputs.buses, results.bus)
        V0 = bus_voltage(bus) * cis(bus_angle(bus))
        state.Vr[1] = real(V0)
        state.Vi[1] = imag(V0)
    end
    return nothing
end

# brings the elements back to the base case, undoing the perturbations of a previous simulation of the same inputs
function reset_element_states!(inputs::Inputs)
    for bus in inputs.buses
        bus.fault_admittance = 0.0 + 0.0im
    end
    for circuit in inputs.circuits
        circuit.in_service = true
    end
    for generator in inputs.generators
        generator.tripped = false
    end
    for ibr in inputs.ibrs
        ibr.tripped = false
    end
    for load in inputs.loads
        load.P = load_P0(load)
        load.Q = load_Q0(load)
    end
    return nothing
end

# the network solution at t = 0 needs the current the IBRs inject, which comes from the power flow
function initialize_ibr_converters_from_power_flow!(inputs::Inputs, results::Results)
    for (ibr, state) in zip(inputs.ibrs, results.ibr)
        initialize_converter_from_power_flow!(inputs, ibr, state)
    end
    return nothing
end

# the controls take Vt, Pmed and Qmed from the network solution at t = 0, the current logic takes Pref and Qref from
# the controls and the converter takes its commands from the current logic
function initialize_ibr_controls!(inputs::Inputs, results::Results)
    for (ibr, state) in zip(inputs.ibrs, results.ibr)
        initialize_active_power_control!(inputs, ibr, state)
        initialize_reactive_power_control!(inputs, ibr, state)
        initialize_current_logic!(inputs, ibr, state)
        initialize_converter!(ibr, state)
    end
    return nothing
end

function initialization(inputs::Inputs)
    validate_ibr_freq_estimations(inputs)

    results = Results(inputs)
    network = get_network(inputs)

    reset_element_states!(inputs)
    initialize_buses!(network, inputs, results)
    initialize_generators!(inputs, results)
    initialize_ibr_converters_from_power_flow!(inputs, results)
    initialize_network!(network, inputs)
    solve_network!(network, inputs, results, 1) # V, Pele, Pmed and Qmed at t = 0 from the initial states
    initialize_mechanical_powers!(inputs, results)
    initialize_voltage_regulators!(inputs, results)
    initialize_bus_freq_estimations!(inputs, results)
    initialize_ibr_controls!(inputs, results)

    return results
end
