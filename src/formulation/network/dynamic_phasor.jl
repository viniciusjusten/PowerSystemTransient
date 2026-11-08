# one state per companion element. The list always has the same elements, so its length is known before the
# initialization
function allocate_companion_states(::NetworkDP, inputs::Inputs, num_steps::Int)
    elements = companion_elements(inputs)
    return CompanionState[CompanionState(num_steps) for _ in elements]
end

# the voltage of a bus at ts, or zero for the ground, which has index 0
function bus_or_ground_voltage(network::NetworkDP, results::Results, bus_idx::Int, ts::Int)
    if bus_idx == 0
        return 0.0 + 0.0im
    end
    bus_state = results.bus[bus_idx]
    return bus_voltage_phasor(network, bus_state, ts)
end

# the voltage across an element at ts, from its sending end to its receiving end
function element_voltage(network::NetworkDP, element::CompanionElement, results::Results, ts::Int)
    from_bus_idx = element_from_bus_idx(element)
    to_bus_idx = element_to_bus_idx(element)
    V_from = bus_or_ground_voltage(network, results, from_bus_idx, ts)
    V_to = bus_or_ground_voltage(network, results, to_bus_idx, ts)
    return V_from - V_to
end

# the tap of an Inductor applies to the voltage of its sending end
function element_voltage(network::NetworkDP, element::Inductor, results::Results, ts::Int)
    from_bus_idx = element_from_bus_idx(element)
    to_bus_idx = element_to_bus_idx(element)
    V_from = bus_or_ground_voltage(network, results, from_bus_idx, ts)
    V_to = bus_or_ground_voltage(network, results, to_bus_idx, ts)
    tap = inductor_tap_ratio(element)
    return tap * V_from - V_to
end

# Steady state currents, with the derivatives at zero. Z = R + j wb L and Y = j wb C, where wb L and wb C are the
# reactance and the susceptance at the base frequency
function steady_state_current(::NetworkDP, element::Inductor, V::ComplexF64)
    Z = inductor_resistance(element) + im * inductor_reactance(element)
    return V / Z
end

function steady_state_current(::NetworkDP, element::Capacitor, V::ComplexF64)
    Y = im * capacitor_susceptance(element)
    return Y * V
end

function steady_state_current(::NetworkDP, element::Resistor, V::ComplexF64)
    return V / resistor_resistance(element)
end

function store_companion_current!(state::CompanionState, I::ComplexF64, ts::Int)
    state.Ir[ts] = real(I)
    state.Ii[ts] = imag(I)
    return nothing
end

# The network starts in steady state, where it is equal to the RMS network, so the solution at t = 0 comes from a
# temporary NetworkRMS, which writes V, id, iq, Pele and the IBR measurements at t = 0. The currents of the elements at
# t = 0 follow from those voltages with the derivatives at zero, and they give the history of the first step
function initialize_network!(network::NetworkDP, inputs::Inputs, results::Results)
    steady_state_network = NetworkRMS()
    initialize_network!(steady_state_network, inputs, results)

    network.elements = companion_elements(inputs)
    elements = network_elements(network)
    for (element, state) in zip(elements, results.companion)
        I = 0.0 + 0.0im
        if element_in_service(element)
            V = element_voltage(network, element, results, 1)
            I = steady_state_current(network, element, V)
        end
        store_companion_current!(state, I, 1)
    end
    return nothing
end
