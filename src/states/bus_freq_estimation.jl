abstract type BusFreqEstimationState <: ControlState end

mutable struct FilteredAngleDerivativeState <: BusFreqEstimationState
    theta_estim::Vector{Float64} # bus voltage angle, the input of the estimator, in rad
    x_theta::Vector{Float64} # state of the filtered derivative, the angle deviation divided by 2 pi fb after the lag 1 / (1 + s Tf), in s
    omega::Vector{Float64} # estimated bus frequency, the output of the low pass filter, in pu
    d_omega::Vector{Float64} # frequency deviation at the output of the filtered derivative, in pu
end

function FilteredAngleDerivativeState(num_steps::Int)
    # NaN keeps a step not yet computed from passing a convergence check by accident
    return FilteredAngleDerivativeState(
        fill(NaN, num_steps),
        fill(NaN, num_steps),
        fill(NaN, num_steps),
        fill(NaN, num_steps),
    )
end

convergence_variables(::FilteredAngleDerivativeState) = [:x_theta, :omega]

function allocate_bus_freq_estimation_state(::FilteredAngleDerivative, num_steps::Int)
    return FilteredAngleDerivativeState(num_steps)
end

# without a frequency estimation there is no state to keep
function allocate_bus_freq_estimation_state(::Nothing, ::Int)
    return nothing
end
