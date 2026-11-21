# Companion models of the alpha beta network, written in the stationary reference frame. The rest of the network
# solution is shared with the other EMT networks, in emt.jl

# The voltages and currents are stored as phasors in the synchronous reference frame, which turns at wb with respect
# to the stationary frame, so a phasor X at the time t is the space vector X e^(j wb t)
function frame_rotation(::NetworkAlphaBeta, inputs::Inputs, ts::Int)
    wb = 2 * pi * fb(inputs)
    t = time_at(inputs, ts)
    return cis(wb * t)
end

# Companion conductances of the implicit trapezoidal rule, with h the integration step, wb = 2 pi fb, L = X / wb and
# C = B / wb. The current of an element is I(t+1) = G V(t+1) + I_hist. The conductances are real in the stationary
# reference frame, but the space vectors are complex, so they are returned as complex values, as in the other EMT
# networks
function companion_conductance(::NetworkAlphaBeta, inputs::Inputs, element::Inductor)
    h = integration_step(inputs)
    wb = 2 * pi * fb(inputs)
    L = inductor_reactance(element) / wb
    R = inductor_resistance(element)
    G = 1 / (2 * L / h + R)
    return complex(G)
end

function companion_conductance(::NetworkAlphaBeta, inputs::Inputs, element::Capacitor)
    h = integration_step(inputs)
    wb = 2 * pi * fb(inputs)
    C = capacitor_susceptance(element) / wb
    G = 2 * C / h
    return complex(G)
end

function companion_conductance(::NetworkAlphaBeta, ::Inputs, element::Resistor)
    G = 1 / resistor_resistance(element)
    return complex(G)
end

# History currents of the implicit trapezoidal rule, from the voltage V and the current I of the element at the previous
# step, with the same h, wb, L and C of the companion conductances
function companion_history_current(::NetworkAlphaBeta, inputs::Inputs, element::Inductor, V::ComplexF64, I::ComplexF64)
    h = integration_step(inputs)
    wb = 2 * pi * fb(inputs)
    L = inductor_reactance(element) / wb
    R = inductor_resistance(element)
    hR_2L = h * R / (2 * L)
    h_2L = h / (2 * L)
    return (1 - hR_2L) * I / (1 + hR_2L) + h_2L * V / (1 + hR_2L)
end

function companion_history_current(::NetworkAlphaBeta, inputs::Inputs, element::Capacitor, V::ComplexF64, I::ComplexF64)
    h = integration_step(inputs)
    wb = 2 * pi * fb(inputs)
    C = capacitor_susceptance(element) / wb
    return -2 * C / h * V - I
end

# the resistor is algebraic, so it has no history
function companion_history_current(::NetworkAlphaBeta, ::Inputs, ::Resistor, ::ComplexF64, ::ComplexF64)
    return 0.0 + 0.0im
end
