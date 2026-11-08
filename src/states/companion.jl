# Current of a companion element, from its sending end to its receiving end. It is the state the companion model
# takes at ts to compute the history of the step to ts + 1
mutable struct CompanionState
    Ir::Vector{Float64} # real part of the element current, in pu
    Ii::Vector{Float64} # imaginary part of the element current, in pu
end

function CompanionState(num_steps::Int)
    # NaN keeps a step not yet computed from being taken as a valid history by accident
    return CompanionState(fill(NaN, num_steps), fill(NaN, num_steps))
end
