function initialize_bus_freq_estimation!(
    inputs::Inputs,
    ::FilteredAngleDerivative,
    estimation_state::FilteredAngleDerivativeState,
    bus_state::BusState,
)
    network = get_network(inputs)
    V0 = bus_voltage_phasor(network, bus_state, 1)
    estimation_state.theta_estim[1] = angle(V0)
    estimation_state.x_theta[1] = 0.0
    estimation_state.omega[1] = 1.0
    estimation_state.d_omega[1] = 0.0
    return nothing
end

# without a frequency estimation there is no state to initialize
function initialize_bus_freq_estimation!(::Inputs, ::Nothing, ::Nothing, ::BusState)
    return nothing
end

function initialize_bus_freq_estimations!(inputs::Inputs, results::Results)
    for (bus, bus_state) in zip(inputs.buses, results.bus)
        freq_estimation = bus_freq_estimation(bus)
        initialize_bus_freq_estimation!(inputs, freq_estimation, bus_state.freq_estimation, bus_state)
    end
    return nothing
end
