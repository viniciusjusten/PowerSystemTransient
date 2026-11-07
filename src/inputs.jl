mutable struct Inputs
    case_path::String

    Sb::Float64 # system base power, in MVA
    fb::Float64 # system base frequency, in Hz

    integration_step::Float64 # integration time step, in s
    simulation_horizon::Float64 # total simulated time, in s
    tolerance::Float64 # convergence tolerance of the numerical solvers
    max_iterations::Int # maximum number of corrector iterations per time step

    buses::Vector{Bus}
    circuits::Vector{Circuit}
    generators::Vector{Generator}
    ibrs::Vector{IBR}
    loads::Vector{Load}
    perturbations::Vector{Perturbation}

    network::Network # representation of the network and its cached matrices
end

function Inputs(;
    case_path::String,
    Sb::Float64,
    fb::Float64,
    integration_step::Float64,
    simulation_horizon::Float64,
    tolerance::Float64,
    max_iterations::Int = 50,
    buses::Vector{Bus} = Bus[],
    circuits::Vector{Circuit} = Circuit[],
    ibrs::Vector{IBR} = IBR[],
    # the vectors of abstract types also accept vectors of a single concrete model, such as [GeneratorTrip(...)],
    # and the struct converts them to the abstract element type
    generators::Vector{<:Generator} = Generator[],
    loads::Vector{<:Load} = Load[],
    perturbations::Vector{<:Perturbation} = Perturbation[],
    network::Network = NetworkRMS(),
)
    return Inputs(
        case_path,
        Sb,
        fb,
        integration_step,
        simulation_horizon,
        tolerance,
        max_iterations,
        buses,
        circuits,
        generators,
        ibrs,
        loads,
        perturbations,
        network,
    )
end

case_path(inputs::Inputs) = inputs.case_path

Sb(inputs::Inputs) = inputs.Sb

fb(inputs::Inputs) = inputs.fb

integration_step(inputs::Inputs) = inputs.integration_step

simulation_horizon(inputs::Inputs) = inputs.simulation_horizon

tolerance(inputs::Inputs) = inputs.tolerance

max_iterations(inputs::Inputs) = inputs.max_iterations

function num_time_steps(inputs::Inputs)
    return round(Int, simulation_horizon(inputs) / integration_step(inputs))
end

# The results are stored from t = 0, so the time index ts holds the instant (ts - 1) * integration_step.
# Every ts passed between functions is this index, and these two functions are the only conversions
function time_at(inputs::Inputs, ts::Int)
    return (ts - 1) * integration_step(inputs)
end

function time_index(inputs::Inputs, time::Float64)
    # round instead of comparing floats, since time / integration_step is rarely an exact integer
    return round(Int, time / integration_step(inputs)) + 1
end

get_network(inputs::Inputs) = inputs.network

get_bus(inputs::Inputs, idx::Int) = inputs.buses[idx]

get_circuit(inputs::Inputs, idx::Int) = inputs.circuits[idx]

get_generator(inputs::Inputs, idx::Int) = inputs.generators[idx]

get_ibr(inputs::Inputs, idx::Int) = inputs.ibrs[idx]

get_load(inputs::Inputs, idx::Int) = inputs.loads[idx]

get_perturbation(inputs::Inputs, idx::Int) = inputs.perturbations[idx]

num_buses(inputs::Inputs) = length(inputs.buses)

num_circuits(inputs::Inputs) = length(inputs.circuits)

num_generators(inputs::Inputs) = length(inputs.generators)

num_ibrs(inputs::Inputs) = length(inputs.ibrs)

num_loads(inputs::Inputs) = length(inputs.loads)

num_perturbations(inputs::Inputs) = length(inputs.perturbations)

function perturbations_at(inputs::Inputs, ts::Int)
    return filter(
        perturbation -> time_index(inputs, perturbation_time(perturbation)) == ts,
        inputs.perturbations,
    )
end
