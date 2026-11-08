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

# the voltage across an element at ts, from its sending end to its receiving end, with the tap applied to the voltage
# of the sending end
function element_voltage(network::NetworkDP, element::CompanionElement, results::Results, ts::Int)
    from_bus_idx = element_from_bus_idx(element)
    to_bus_idx = element_to_bus_idx(element)
    V_from = bus_or_ground_voltage(network, results, from_bus_idx, ts)
    V_to = bus_or_ground_voltage(network, results, to_bus_idx, ts)
    tap = element_tap_ratio(element)
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

# Companion conductances of the implicit trapezoidal rule, with h the integration step, wb = 2 pi fb, L = X / wb and
# C = B / wb. The current of an element is I(t+1) = G V(t+1) + I_hist
function companion_conductance(::NetworkDP, inputs::Inputs, element::Inductor)
    h = integration_step(inputs)
    wb = 2 * pi * fb(inputs)
    L = inductor_reactance(element) / wb
    Z = inductor_resistance(element) + im * wb * L
    return 1 / (2 * L / h + Z)
end

function companion_conductance(::NetworkDP, inputs::Inputs, element::Capacitor)
    h = integration_step(inputs)
    wb = 2 * pi * fb(inputs)
    C = capacitor_susceptance(element) / wb
    Y = im * wb * C
    return 2 * C / h + Y
end

# the conductances of the dynamic phasor network are complex, so the resistor returns a complex value as well
function companion_conductance(::NetworkDP, ::Inputs, element::Resistor)
    G = 1 / resistor_resistance(element)
    return complex(G)
end

# History currents of the implicit trapezoidal rule, from the voltage V and the current I of the element at the previous
# step, with the same h, wb, L and C of the companion conductances
function companion_history_current(::NetworkDP, inputs::Inputs, element::Inductor, V::ComplexF64, I::ComplexF64)
    h = integration_step(inputs)
    wb = 2 * pi * fb(inputs)
    L = inductor_reactance(element) / wb
    Z = inductor_resistance(element) + im * wb * L
    Zh_2L = Z * h / (2 * L)
    h_2L = h / (2 * L)
    return (1 - Zh_2L) * I / (1 + Zh_2L) + h_2L * V / (1 + Zh_2L)
end

function companion_history_current(::NetworkDP, inputs::Inputs, element::Capacitor, V::ComplexF64, I::ComplexF64)
    h = integration_step(inputs)
    wb = 2 * pi * fb(inputs)
    C = capacitor_susceptance(element) / wb
    Y = im * wb * C
    Yh_2C = Y * h / (2 * C)
    h_2C = h / (2 * C)
    return -I - (1 - Yh_2C) * V / h_2C
end

# the resistor is algebraic, so it has no history
function companion_history_current(::NetworkDP, ::Inputs, ::Resistor, ::ComplexF64, ::ComplexF64)
    return 0.0 + 0.0im
end

# the positions an element with companion conductance G takes in the matrix, with the tap applied to the sending end,
# so the current leaving the sending end is tap * I. An element to the ground only takes the diagonal of its sending
# end. Several elements share positions, such as the shunts at a bus, and sparse sums the repeated positions
function push_companion_stamp!(
    rows::Vector{Int},
    cols::Vector{Int},
    values::Vector{ComplexF64},
    element::CompanionElement,
    G::ComplexF64,
)
    from = element_from_bus_idx(element)
    to = element_to_bus_idx(element)
    tap = element_tap_ratio(element)
    push!(rows, from)
    push!(cols, from)
    push!(values, tap^2 * G)
    if to != 0
        append!(rows, [to, from, to])
        append!(cols, [to, to, from])
        append!(values, [G, -tap * G, -tap * G])
    end
    return nothing
end

# The conductance matrix of the companion models, assembled from (row, column, value) triplets as in admittance_matrix.
# Elements out of service and stators of tripped generators still contribute zeros at their positions, so the sparsity
# pattern never changes and the symbolic factorization of the LU stays valid after a perturbation
function companion_matrix(network::NetworkDP, inputs::Inputs)
    n_buses = num_buses(inputs)
    rows = Int[]
    cols = Int[]
    values = ComplexF64[]

    elements = network_elements(network)
    for element in elements
        G = 0.0 + 0.0im
        if element_in_service(element)
            G = companion_conductance(network, inputs, element)
        end
        push_companion_stamp!(rows, cols, values, element, G)
    end

    for generator in inputs.generators
        stator = generator_stator(inputs, generator)
        G = 0.0 + 0.0im
        if element_in_service(stator)
            G = companion_conductance(network, inputs, stator)
        end
        push_companion_stamp!(rows, cols, values, stator, G)
    end

    return sparse(rows, cols, values, n_buses, n_buses)
end

function store_companion_current!(state::CompanionState, I::ComplexF64, ts::Int)
    state.Ir[ts] = real(I)
    state.Ii[ts] = imag(I)
    return nothing
end

# The network starts in steady state, where it is equal to the RMS network, so the solution at t = 0 comes from a
# temporary NetworkRMS, which writes V, id, iq, Pele and the IBR measurements at t = 0. The currents of the elements at
# t = 0 follow from those voltages with the derivatives at zero, and they give the history of the first step. The
# companion matrix and its factorization are built last, for the steps that follow
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

    G = companion_matrix(network, inputs)
    network.G = G
    network.G_lu = lu(G)
    return nothing
end

# the perturbations only change the state of the elements, so the list of elements and the companion matrix are rebuilt
# from that state, with the same sparsity pattern
function rebuild_network!(network::NetworkDP, inputs::Inputs)
    network.elements = companion_elements(inputs)
    G = companion_matrix(network, inputs)
    network.G = G
    G_lu = network_G_lu(network)
    lu!(G_lu, G)
    return nothing
end
