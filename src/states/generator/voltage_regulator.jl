abstract type VoltageRegulatorState <: ControlState end

mutable struct Static1stOrderState <: VoltageRegulatorState
    efdu::Vector{Float64} # excitation voltage before the limiter, in pu
    Vref::Vector{Float64} # reference voltage of the voltage regulator, in pu
end

function Static1stOrderState(num_steps::Int)
    # NaN keeps a step not yet computed from passing a convergence check by accident
    return Static1stOrderState(fill(NaN, num_steps), fill(NaN, num_steps))
end

function allocate_voltage_regulator_state(::Static1stOrder, num_steps::Int)
    return Static1stOrderState(num_steps)
end

# without a voltage regulator there is no state to keep
function allocate_voltage_regulator_state(::Nothing, ::Int)
    return nothing
end
