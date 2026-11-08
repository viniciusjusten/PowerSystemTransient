# A shunt admittance Y = G + jB is a Resistor for the conductance, in parallel with a Capacitor for a positive
# susceptance or with an Inductor without resistance for a negative one. A part that is zero adds no element
function push_shunt_elements!(
    elements::Vector{CompanionElement},
    name::String,
    bus_idx::Int,
    conductance::Float64,
    susceptance::Float64,
    in_service::Bool,
)
    if conductance != 0
        resistor = Resistor(;
            name = "$(name)_resistor",
            from_bus_idx = bus_idx,
            to_bus_idx = 0,
            resistance = 1 / conductance,
            in_service = in_service,
        )
        push!(elements, resistor)
    end
    if susceptance > 0
        capacitor = Capacitor(;
            name = "$(name)_capacitor",
            from_bus_idx = bus_idx,
            to_bus_idx = 0,
            susceptance = susceptance,
            in_service = in_service,
        )
        push!(elements, capacitor)
    elseif susceptance < 0
        inductor = Inductor(;
            name = "$(name)_inductor",
            from_bus_idx = bus_idx,
            to_bus_idx = 0,
            resistance = 0.0,
            reactance = -1 / susceptance,
            in_service = in_service,
        )
        push!(elements, inductor)
    end
    return nothing
end

# The series branch of the pi model is an Inductor with the tap of the circuit, and each end gets half of the shunt
function push_circuit_elements!(elements::Vector{CompanionElement}, circuit::Circuit)
    name = circuit_name(circuit)
    from = circuit_from_bus_idx(circuit)
    to = circuit_to_bus_idx(circuit)
    in_service = circuit_in_service(circuit)

    series = Inductor(;
        name = "circuit_$(name)_series",
        from_bus_idx = from,
        to_bus_idx = to,
        resistance = circuit_resistance(circuit),
        reactance = circuit_reactance(circuit),
        tap_ratio = circuit_tap_ratio(circuit),
        in_service = in_service,
    )
    push!(elements, series)

    shunt_conductance = circuit_shunt_conductance(circuit) / 2
    shunt_susceptance = circuit_shunt_susceptance(circuit) / 2
    push_shunt_elements!(elements, "circuit_$(name)_shunt_from", from, shunt_conductance, shunt_susceptance, in_service)
    push_shunt_elements!(elements, "circuit_$(name)_shunt_to", to, shunt_conductance, shunt_susceptance, in_service)
    return nothing
end

# The fault Resistor is always in the list, so the list keeps its length when a short circuit is applied or removed.
# It is in service only while the fault conductance is not zero, and without a fault its resistance is infinite
function push_bus_elements!(elements::Vector{CompanionElement}, bus_idx::Int, bus::Bus)
    name = bus_name(bus)

    shunt_susceptance = bus_shunt_susceptance(bus)
    push_shunt_elements!(elements, "bus_$(name)_shunt", bus_idx, 0.0, shunt_susceptance, true)

    fault_conductance = real(bus_fault_admittance(bus))
    fault = Resistor(;
        name = "bus_$(name)_fault",
        from_bus_idx = bus_idx,
        to_bus_idx = 0,
        resistance = 1 / fault_conductance,
        in_service = fault_conductance != 0,
    )
    push!(elements, fault)
    return nothing
end

# The constant impedance load draws Y = (P - jQ) / V^2 at the power flow voltage of its bus
function push_load_elements!(elements::Vector{CompanionElement}, inputs::Inputs, load::ConstantZLoad)
    name = load_name(load)
    bus_idx = load_bus(load)
    bus = get_bus(inputs, bus_idx)
    V = bus_voltage(bus)
    conductance = load_P(load) / V^2
    susceptance = -load_Q(load) / V^2
    push_shunt_elements!(elements, "load_$(name)", bus_idx, conductance, susceptance, true)
    return nothing
end

# The companion elements of the network, built from the current state of the circuits, buses and loads. The stators of
# the generators also have companion models, but they are not in this list, since their currents are the states id and
# iq of the generators and one of their ends is the internal EMF instead of a bus. The list always has the same
# elements in the same order, and an element out of service stays in it, so the sparsity pattern of the companion
# matrix and the index of each element state do not change after a perturbation
function companion_elements(inputs::Inputs)
    elements = CompanionElement[]
    for circuit in inputs.circuits
        push_circuit_elements!(elements, circuit)
    end
    for (bus_idx, bus) in enumerate(inputs.buses)
        push_bus_elements!(elements, bus_idx, bus)
    end
    for load in inputs.loads
        push_load_elements!(elements, inputs, load)
    end
    return elements
end
