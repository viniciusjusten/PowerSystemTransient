# Companion models of the dynamic phasor network, written in the synchronous reference frame. The rest of the network
# solution is shared with the other EMT networks, in emt.jl

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
