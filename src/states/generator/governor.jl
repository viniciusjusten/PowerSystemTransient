abstract type GovernorState <: ControlState end

mutable struct Droop1stOrderState <: GovernorState
    Pmec_1::Vector{Float64} # mechanical power after the regulator and valves, in pu
    Pref::Vector{Float64} # reference power of the governor, in pu
end

function Droop1stOrderState(num_steps::Int)
    # NaN keeps a step not yet computed from passing a convergence check by accident
    return Droop1stOrderState(fill(NaN, num_steps), fill(NaN, num_steps))
end

function allocate_governor_state(::Droop1stOrder, num_steps::Int)
    return Droop1stOrderState(num_steps)
end

# without a governor there is no state to keep
function allocate_governor_state(::Nothing, ::Int)
    return nothing
end
