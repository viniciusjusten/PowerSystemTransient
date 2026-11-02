mutable struct BusState
    Vr::Vector{Float64} # real part of the bus voltage, in pu
    Vi::Vector{Float64} # imaginary part of the bus voltage, in pu

    # Measurements
    freq_estimation::Union{Nothing, BusFreqEstimationState} # state of the frequency estimation, nothing when it is not estimated
end

function BusState(num_steps::Int, freq_estimation::Union{Nothing, BusFreqEstimationState})
    # NaN keeps a step not yet computed from passing a convergence check by accident
    return BusState(fill(NaN, num_steps), fill(NaN, num_steps), freq_estimation)
end

# the voltages are algebraic, but they come from the network solution that the corrector iterates
convergence_variables(::BusState) = [:Vr, :Vi]

function allocate_bus_state(bus::Bus, num_steps::Int)
    freq_estimation = allocate_bus_freq_estimation_state(bus_freq_estimation(bus), num_steps)
    return BusState(num_steps, freq_estimation)
end
