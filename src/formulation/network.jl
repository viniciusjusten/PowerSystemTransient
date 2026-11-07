function admittance(resistance::Float64, reactance::Float64)
    return 1 / (resistance + im * reactance)
end

# builds the admittance matrix from the current state of the elements, including generators and loads
function admittance_matrix(::NetworkRMS, inputs::Inputs)
    n_buses = num_buses(inputs)

    # the matrix is assembled from (row, column, value) triplets, and sparse sums repeated positions.
    # Elements out of service still contribute zero values at their positions, so the sparsity pattern
    # never changes and the symbolic factorization of the LU stays valid after a perturbation
    rows = Int[]
    cols = Int[]
    values = ComplexF64[]
    for circuit in inputs.circuits
        from = circuit_from_bus_idx(circuit)
        to = circuit_to_bus_idx(circuit)
        tap = circuit_tap_ratio(circuit)
        y = 0.0 + 0.0im
        y_shunt = 0.0 + 0.0im
        if circuit_in_service(circuit)
            y = admittance(circuit_resistance(circuit), circuit_reactance(circuit))
            # the circuit shunt is the total of the pi model, so each end gets half of it
            y_shunt = circuit_shunt_conductance(circuit) + im * circuit_shunt_susceptance(circuit)
        end
        append!(rows, [from, to, from, to])
        append!(cols, [from, to, to, from])
        append!(values, [y * tap^2 + y_shunt / 2, y + y_shunt / 2, -y * tap, -y * tap])
    end

    for (bus_idx, bus) in enumerate(inputs.buses)
        push!(rows, bus_idx)
        push!(cols, bus_idx)
        push!(values, im * bus_shunt_susceptance(bus) + bus_fault_admittance(bus))
    end

    for generator in inputs.generators
        if !generator_tripped(generator)
            bus_idx = generator_bus(generator)
            push!(rows, bus_idx)
            push!(cols, bus_idx)
            push!(values, generator_admittance(inputs, generator))
        end
    end

    for load in inputs.loads
        bus_idx = load_bus(load)
        push!(rows, bus_idx)
        push!(cols, bus_idx)
        push!(values, load_admittance(inputs, load))
    end

    return sparse(rows, cols, values, n_buses, n_buses)
end

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

# The current vector I only includes the generators and IBRs that are in service, since the loads are included in the
# admittance matrix
function current_injections(::PhasorNetwork, inputs::Inputs, results::Results, ts::Int)
    I = zeros(ComplexF64, num_buses(inputs))
    for (generator, state) in zip(inputs.generators, results.generator)
        if !generator_tripped(generator)
            I[generator_bus(generator)] += generator_current_network_contribution(inputs, results, generator, state, ts)
        end
    end
    for (ibr, state) in zip(inputs.ibrs, results.ibr)
        if !ibr_tripped(ibr)
            bus_idx = ibr_bus(ibr)
            I[bus_idx] += ibr_current_network_contribution(inputs, results, ibr, state, ts)
        end
    end
    return I
end

# the currents, terminal voltages, electrical powers and IBR measurements at ts follow from the bus voltages at ts,
# whatever the formulation of the network
function update_device_algebraic_variables!(inputs::Inputs, results::Results, ts::Int)
    # the currents come first because the electrical power of the OneDOneQ generator is computed from them
    for (generator, state) in zip(inputs.generators, results.generator)
        state.id[ts], state.iq[ts] = generator_current_dq(inputs, results, generator, state, ts)
        state.Vt[ts] = terminal_voltage(inputs, results, generator, ts)
        state.Pele[ts] = electrical_power(inputs, results, generator, state, ts)
    end

    for (ibr, state) in zip(inputs.ibrs, results.ibr)
        update_ibr_measurements!(inputs, results, ibr, state, ts)
    end

    return nothing
end

function initialize_network!(network::NetworkRMS, inputs::Inputs)
    Ybus = admittance_matrix(network, inputs)
    network.Ybus_cache = Ybus
    network.Ybus_lu = lu(Ybus)
    return nothing
end

# the perturbations only change the state of the elements, and the admittance matrix is rebuilt from that state
function rebuild_network!(network::NetworkRMS, inputs::Inputs)
    Ybus = admittance_matrix(network, inputs)
    network.Ybus_cache = Ybus
    Ybus_lu = network_Ybus_lu(network)
    lu!(Ybus_lu, Ybus)
    return nothing
end

# solves the network with the generator and IBR states at ts and writes the voltages, currents, electrical powers and
# IBR measurements at ts
function solve_network!(network::NetworkRMS, inputs::Inputs, results::Results, ts::Int)
    I = current_injections(network, inputs, results, ts)
    Ybus_lu = network_Ybus_lu(network)
    V = Ybus_lu \ I
    store_bus_voltages!(network, results, V, ts)
    update_device_algebraic_variables!(inputs, results, ts)
    return nothing
end
