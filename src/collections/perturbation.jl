abstract type Perturbation end

mutable struct ShortCircuit <: Perturbation
    name::String
    bus_idx::Int # index of the bus where the fault is applied

    time::Float64 # instant the fault is applied, in s

    fault_conductance::Float64 # shunt conductance added by the fault, in pu
    fault_susceptance::Float64 # shunt susceptance added by the fault, in pu
end

function ShortCircuit(;
    name::String,
    bus_idx::Int,
    time::Float64,
    fault_conductance::Float64,
    fault_susceptance::Float64,
)
    return ShortCircuit(
        name,
        bus_idx,
        time,
        fault_conductance,
        fault_susceptance,
    )
end

perturbation_name(perturbation::Perturbation) = perturbation.name

perturbation_bus_idx(perturbation::ShortCircuit) = perturbation.bus_idx

perturbation_time(perturbation::Perturbation) = perturbation.time

perturbation_fault_conductance(perturbation::ShortCircuit) = perturbation.fault_conductance

perturbation_fault_susceptance(perturbation::ShortCircuit) = perturbation.fault_susceptance

function apply_perturbation!(inputs, perturbation::ShortCircuit)
    bus_idx = perturbation_bus_idx(perturbation)
    inputs.buses[bus_idx].fault_admittance =
        perturbation_fault_conductance(perturbation) + im * perturbation_fault_susceptance(perturbation)
    return nothing
end

mutable struct ShortCircuitRemoval <: Perturbation
    name::String
    short_circuit_idx::Int # index in inputs.perturbations of the ShortCircuit that is removed

    time::Float64 # instant the fault is cleared, in s
end

function ShortCircuitRemoval(;
    name::String,
    short_circuit_idx::Int,
    time::Float64,
)
    return ShortCircuitRemoval(name, short_circuit_idx, time)
end

perturbation_short_circuit_idx(perturbation::ShortCircuitRemoval) = perturbation.short_circuit_idx

function apply_perturbation!(inputs, perturbation::ShortCircuitRemoval)
    short_circuit = get_perturbation(inputs, perturbation_short_circuit_idx(perturbation))
    inputs.buses[perturbation_bus_idx(short_circuit)].fault_admittance = 0.0 + 0.0im
    return nothing
end

mutable struct CircuitTrip <: Perturbation
    name::String
    circuit_idx::Int # index of the circuit that is tripped

    time::Float64 # instant the circuit is tripped, in s
end

function CircuitTrip(;
    name::String,
    circuit_idx::Int,
    time::Float64,
)
    return CircuitTrip(name, circuit_idx, time)
end

perturbation_circuit_idx(perturbation::CircuitTrip) = perturbation.circuit_idx

function apply_perturbation!(inputs, perturbation::CircuitTrip)
    inputs.circuits[perturbation_circuit_idx(perturbation)].in_service = false
    return nothing
end

mutable struct GeneratorTrip <: Perturbation
    name::String
    generator_idx::Int # index of the generator that is tripped

    time::Float64 # instant the generator is tripped, in s
end

function GeneratorTrip(;
    name::String,
    generator_idx::Int,
    time::Float64,
)
    return GeneratorTrip(name, generator_idx, time)
end

perturbation_generator_idx(perturbation::GeneratorTrip) = perturbation.generator_idx

function apply_perturbation!(inputs, perturbation::GeneratorTrip)
    inputs.generators[perturbation_generator_idx(perturbation)].tripped = true
    return nothing
end

mutable struct IBRTrip <: Perturbation
    name::String
    ibr_idx::Int # index of the IBR that is tripped

    time::Float64 # instant the IBR is tripped, in s
end

function IBRTrip(;
    name::String,
    ibr_idx::Int,
    time::Float64,
)
    return IBRTrip(name, ibr_idx, time)
end

perturbation_ibr_idx(perturbation::IBRTrip) = perturbation.ibr_idx

function apply_perturbation!(inputs, perturbation::IBRTrip)
    inputs.ibrs[perturbation_ibr_idx(perturbation)].tripped = true
    return nothing
end

mutable struct LoadChange <: Perturbation
    name::String
    load_idx::Int # index of the load that changes

    time::Float64 # instant the load changes, in s

    new_p0::Float64 # new active power at the power flow voltage of the load bus, in pu
    new_q0::Float64 # new reactive power at the power flow voltage of the load bus, in pu
end

function LoadChange(;
    name::String,
    load_idx::Int,
    time::Float64,
    new_p0::Float64,
    new_q0::Float64,
)
    return LoadChange(name, load_idx, time, new_p0, new_q0)
end

perturbation_load_idx(perturbation::LoadChange) = perturbation.load_idx

perturbation_new_p0(perturbation::LoadChange) = perturbation.new_p0

perturbation_new_q0(perturbation::LoadChange) = perturbation.new_q0

function apply_perturbation!(inputs, perturbation::LoadChange)
    load = inputs.loads[perturbation_load_idx(perturbation)]
    load.P = perturbation_new_p0(perturbation)
    load.Q = perturbation_new_q0(perturbation)
    return nothing
end
