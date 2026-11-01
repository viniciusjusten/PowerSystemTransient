abstract type GeneratorState end

mutable struct ClassicGeneratorState <: GeneratorState
    delta::Vector{Float64} # rotor angle, in rad
    omega::Vector{Float64} # rotor speed, in pu
    Pmec::Vector{Float64} # mechanical power applied to the rotor, in pu
    Pele::Vector{Float64} # electrical power, in pu
    Eq_prime::Float64 # internal EMF magnitude, in pu
    id::Vector{Float64} # stator current along the d-axis, in pu
    iq::Vector{Float64} # stator current along the q-axis, in pu
    Vt::Vector{Float64} # terminal voltage, in pu

    # Controls
    governor::Union{Nothing, GovernorState} # state of the speed governor, nothing when there is no governor
    voltage_regulator::Nothing # classic generators do not have a voltage regulator. This is kept for the multiple dispatch to work
end

function ClassicGeneratorState(num_steps::Int, governor::Union{Nothing, GovernorState})
    # NaN keeps a step not yet computed from passing a convergence check by accident
    # Eq_prime is set once initialization computes it
    return ClassicGeneratorState(
        fill(NaN, num_steps),
        fill(NaN, num_steps),
        fill(NaN, num_steps),
        fill(NaN, num_steps),
        NaN,
        fill(NaN, num_steps),
        fill(NaN, num_steps),
        fill(NaN, num_steps),
        governor,
        nothing,
    )
end

function allocate_generator_state(generator::ClassicGenerator, num_steps::Int)
    governor = allocate_governor_state(generator_governor(generator), num_steps)
    return ClassicGeneratorState(num_steps, governor)
end

mutable struct OneDOneQGeneratorState <: GeneratorState
    delta::Vector{Float64} # rotor angle, in rad
    omega::Vector{Float64} # rotor speed, in pu
    Pmec::Vector{Float64} # mechanical power applied to the rotor, in pu
    Pele::Vector{Float64} # electrical power, in pu
    ed_prime::Vector{Float64} # internal EMF along the d-axis, in pu
    eq_prime::Vector{Float64} # internal EMF along the q-axis, in pu
    id::Vector{Float64} # stator current along the d-axis, in pu
    iq::Vector{Float64} # stator current along the q-axis, in pu
    efd::Vector{Float64} # excitation voltage applied to the field, in pu
    Vt::Vector{Float64} # terminal voltage, in pu

    # Controls
    governor::Union{Nothing, GovernorState} # state of the speed governor, nothing when there is no governor
    voltage_regulator::Union{Nothing, VoltageRegulatorState} # state of the voltage regulator, nothing when there is no voltage regulator
end

function OneDOneQGeneratorState(
    num_steps::Int,
    governor::Union{Nothing, GovernorState},
    voltage_regulator::Union{Nothing, VoltageRegulatorState},
)
    # NaN keeps a step not yet computed from passing a convergence check by accident
    return OneDOneQGeneratorState(
        fill(NaN, num_steps),
        fill(NaN, num_steps),
        fill(NaN, num_steps),
        fill(NaN, num_steps),
        fill(NaN, num_steps),
        fill(NaN, num_steps),
        fill(NaN, num_steps),
        fill(NaN, num_steps),
        fill(NaN, num_steps),
        fill(NaN, num_steps),
        governor,
        voltage_regulator,
    )
end

function allocate_generator_state(generator::OneDOneQGenerator, num_steps::Int)
    governor = allocate_governor_state(generator_governor(generator), num_steps)
    voltage_regulator = allocate_voltage_regulator_state(generator_voltage_regulator(generator), num_steps)
    return OneDOneQGeneratorState(num_steps, governor, voltage_regulator)
end
