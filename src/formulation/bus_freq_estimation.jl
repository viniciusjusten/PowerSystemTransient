# Filtered angle derivative frequency estimation model
function deriv_freq_estim_x_theta(
    inputs::Inputs,
    bus_freq_estimation::FilteredAngleDerivative,
    bus_freq_estimation_state::FilteredAngleDerivativeState,
    ts::Int,
)
    # bus_freq_estimation_state.theta_estim[1] is the initial bus voltage angle, which is used as a reference for the frequency estimation
    return 1 / bus_freq_estimation_Tf(bus_freq_estimation) * (
        (bus_freq_estimation_state.theta_estim[ts] - bus_freq_estimation_state.theta_estim[1]) /
        (2 * pi * fb(inputs)) - bus_freq_estimation_state.x_theta[ts]
    )
end

function deriv_freq_estim_omega(
    bus_freq_estimation::FilteredAngleDerivative,
    bus_freq_estimation_state::FilteredAngleDerivativeState,
    ts::Int,
)
    return 1 / bus_freq_estimation_Tw(bus_freq_estimation) * (
        bus_freq_estimation_state.d_omega[ts] + 1 - bus_freq_estimation_state.omega[ts]
    )
end

# the bus voltage angle at ts, taken in the 2 pi interval around the last estimated angle, so it stays continuous
# when it crosses pi
function continuous_bus_angle(
    inputs::Inputs,
    bus_state::BusState,
    bus_freq_estimation_state::FilteredAngleDerivativeState,
    ts::Int,
)
    network = get_network(inputs)
    V = bus_voltage_phasor(network, bus_state, ts)
    wrapped_angle = angle(V)
    angle_difference = rem2pi(wrapped_angle - bus_freq_estimation_state.theta_estim[ts-1], RoundNearest)
    return bus_freq_estimation_state.theta_estim[ts-1] + angle_difference
end

function predict_bus_freq_estimation!(
    inputs::Inputs,
    bus_freq_estimation::FilteredAngleDerivative,
    bus_freq_estimation_state::FilteredAngleDerivativeState,
    bus_state::BusState,
    ts::Int,
)
    # a perturbation at ts solves the network again, so the angle and d_omega at ts are updated before the
    # derivatives at ts. x_theta and omega are states and keep their values. At ts = 1 there is no previous angle
    # to keep the angle continuous, and theta_estim[1] is the reference angle, so a perturbation at t = 0 is seen
    # as a ramp over the first step
    if ts > 1
        bus_freq_estimation_state.theta_estim[ts] =
            continuous_bus_angle(inputs, bus_state, bus_freq_estimation_state, ts)
        bus_freq_estimation_state.d_omega[ts] =
            deriv_freq_estim_x_theta(inputs, bus_freq_estimation, bus_freq_estimation_state, ts)
    end

    deriv_x_theta = deriv_freq_estim_x_theta(inputs, bus_freq_estimation, bus_freq_estimation_state, ts)
    deriv_omega = deriv_freq_estim_omega(bus_freq_estimation, bus_freq_estimation_state, ts)

    bus_freq_estimation_state.theta_estim[ts+1] =
        continuous_bus_angle(inputs, bus_state, bus_freq_estimation_state, ts+1)

    bus_freq_estimation_state.x_theta[ts+1] =
        bus_freq_estimation_state.x_theta[ts] +
        integration_step(inputs) * deriv_x_theta

    # Δω is the derivative of x_theta, so it is computed at the next step using the derivative function
    bus_freq_estimation_state.d_omega[ts+1] =
        deriv_freq_estim_x_theta(inputs, bus_freq_estimation, bus_freq_estimation_state, ts+1)

    bus_freq_estimation_state.omega[ts+1] =
        bus_freq_estimation_state.omega[ts] +
        integration_step(inputs) * deriv_omega

    return nothing
end

function step_bus_freq_estimation!(
    inputs::Inputs,
    bus_freq_estimation::FilteredAngleDerivative,
    bus_freq_estimation_state::FilteredAngleDerivativeState,
    bus_state::BusState,
    ts::Int,
)
    deriv_x_theta = deriv_freq_estim_x_theta(inputs, bus_freq_estimation, bus_freq_estimation_state, ts)
    deriv_omega = deriv_freq_estim_omega(bus_freq_estimation, bus_freq_estimation_state, ts)

    bus_freq_estimation_state.theta_estim[ts+1] =
        continuous_bus_angle(inputs, bus_state, bus_freq_estimation_state, ts+1)

    bus_freq_estimation_state.x_theta[ts+1] =
        bus_freq_estimation_state.x_theta[ts] +
        integration_step(inputs) / 2 * (
            deriv_x_theta +
            deriv_freq_estim_x_theta(inputs, bus_freq_estimation, bus_freq_estimation_state, ts+1)
        )

    # Δω is the derivative of x_theta, so it is computed at the next step using the derivative function
    bus_freq_estimation_state.d_omega[ts+1] =
        deriv_freq_estim_x_theta(inputs, bus_freq_estimation, bus_freq_estimation_state, ts+1)

    bus_freq_estimation_state.omega[ts+1] =
        bus_freq_estimation_state.omega[ts] +
        integration_step(inputs) / 2 * (
            deriv_omega +
            deriv_freq_estim_omega(bus_freq_estimation, bus_freq_estimation_state, ts+1)
        )

    return nothing
end

# without a frequency estimation there is no state to keep
function predict_bus_freq_estimation!(::Inputs, ::Nothing, ::Nothing, ::BusState, ::Int)
    return nothing
end

function step_bus_freq_estimation!(::Inputs, ::Nothing, ::Nothing, ::BusState, ::Int)
    return nothing
end
