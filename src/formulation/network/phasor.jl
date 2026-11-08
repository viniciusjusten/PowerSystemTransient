# The bus voltages of the phasor networks are complex phasors in the synchronous reference frame, stored as Vr and Vi
# in the bus states. The devices read them only through this function, so a formulation whose voltages are not
# phasors gives its own interface here
function bus_voltage_phasor(::PhasorNetwork, bus_state::BusState, ts::Int)
    return bus_state.Vr[ts] + im * bus_state.Vi[ts]
end

function store_bus_voltages!(::PhasorNetwork, results::Results, V::Vector{ComplexF64}, ts::Int)
    for (bus_idx, state) in enumerate(results.bus)
        state.Vr[ts] = real(V[bus_idx])
        state.Vi[ts] = imag(V[bus_idx])
    end
    return nothing
end

# the currents, terminal voltages, electrical powers and IBR measurements at ts follow from the bus voltages at ts,
# whatever the formulation of the network
function update_device_algebraic_variables!(network::Network, inputs::Inputs, results::Results, ts::Int)
    # the currents come first because the electrical power of the OneDOneQ generator is computed from them
    for (generator, state) in zip(inputs.generators, results.generator)
        state.id[ts], state.iq[ts] = generator_current_dq(network, inputs, results, generator, state, ts)
        state.Vt[ts] = terminal_voltage(inputs, results, generator, ts)
        state.Pele[ts] = electrical_power(inputs, generator, state, ts)
    end

    for (ibr, state) in zip(inputs.ibrs, results.ibr)
        update_ibr_measurements!(inputs, results, ibr, state, ts)
    end

    return nothing
end
