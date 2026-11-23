abstract type Network end

# Networks whose bus voltages are complex phasors in the synchronous reference frame, so the devices read their
# voltages in the same way in all of them
abstract type PhasorNetwork <: Network end

# Phasor (RMS) representation of the network, solved algebraically as Ybus * V = I at each time step
mutable struct NetworkRMS <: PhasorNetwork
    # Cache
    Ybus_cache::SparseMatrixCSC{ComplexF64, Int} # admittance matrix of the current state of the elements, rebuilt at each event
    Ybus_lu::Union{Nothing, SparseArrays.UMFPACK.UmfpackLU{ComplexF64, Int}} # LU factorization of Ybus_cache, refactored at each event
end

# The admittance matrix and its factorization are built at the initialization, so the network starts empty
function NetworkRMS()
    return NetworkRMS(spzeros(ComplexF64, 0, 0), nothing)
end

network_Ybus_cache(network::NetworkRMS) = network.Ybus_cache

network_Ybus_lu(network::NetworkRMS) = network.Ybus_lu

# Networks whose inductances and capacitances have states, which the implicit trapezoidal rule turns into companion
# models, so the network is solved as G * V = rhs at each time step. The formulations differ only in the companion
# equations and in the reference frame in which the network is solved. The bus voltages are still stored as phasors in
# the synchronous reference frame, so these networks are also phasor networks
abstract type EMTNetwork <: PhasorNetwork end

# Dynamic phasor representation of the network, solved in the synchronous reference frame
mutable struct NetworkDP <: EMTNetwork
    # Cache
    elements::Vector{CompanionElement} # companion elements of the circuits, buses and loads, rebuilt at each event. The generator stators are not included
    G::SparseMatrixCSC{ComplexF64, Int} # conductance matrix of the companion models, rebuilt at each event
    G_lu::Union{Nothing, SparseArrays.UMFPACK.UmfpackLU{ComplexF64, Int}} # LU factorization of G, refactored at each event
    rhs::Vector{ComplexF64} # device injections minus the history currents of the elements, rebuilt at each solution
end

# The elements, the matrix and its factorization are built at the initialization, so the network starts empty
function NetworkDP()
    return NetworkDP(CompanionElement[], spzeros(ComplexF64, 0, 0), nothing, ComplexF64[])
end

# Electromagnetic transient representation of a balanced network, solved in the stationary reference frame, where the
# space vector V_alpha + j V_beta stands for the three phases
mutable struct NetworkAlphaBeta <: EMTNetwork
    # Cache
    elements::Vector{CompanionElement} # companion elements of the circuits, buses and loads, rebuilt at each event. The generator stators are not included
    G::SparseMatrixCSC{ComplexF64, Int} # conductance matrix of the companion models, rebuilt at each event
    G_lu::Union{Nothing, SparseArrays.UMFPACK.UmfpackLU{ComplexF64, Int}} # LU factorization of G, refactored at each event
    rhs::Vector{ComplexF64} # device injections minus the history currents of the elements, rebuilt at each solution
end

# The elements, the matrix and its factorization are built at the initialization, so the network starts empty
function NetworkAlphaBeta()
    return NetworkAlphaBeta(CompanionElement[], spzeros(ComplexF64, 0, 0), nothing, ComplexF64[])
end

network_elements(network::EMTNetwork) = network.elements

network_G_lu(network::EMTNetwork) = network.G_lu

network_rhs(network::EMTNetwork) = network.rhs
