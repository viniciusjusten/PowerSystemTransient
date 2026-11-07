# Each state type lists in convergence_variables the variables the corrector checks, and the control states nested in
# a state, such as the governor of a generator or the frequency estimation of a bus, list their own. A control that is
# absent is nothing, which is not a ControlState, so it adds no values
function push_convergence_values!(values::Vector{Float64}, state, ts::Int)
    for field in convergence_variables(state)
        series = getfield(state, field)
        push!(values, series[ts])
    end
    for field in fieldnames(typeof(state))
        value = getfield(state, field)
        if value isa ControlState
            push_convergence_values!(values, value, ts)
        end
    end
    return nothing
end

# the values at ts of the variables the corrector checks, always collected in the same order, so the values of two
# iterations can be compared position by position
function convergence_values(results::Results, ts::Int)
    values = Float64[]
    for state in results.generator
        push_convergence_values!(values, state, ts)
    end
    for state in results.bus
        push_convergence_values!(values, state, ts)
    end
    for state in results.ibr
        push_convergence_values!(values, state, ts)
    end
    return values
end

function predict_voltages!(::PhasorNetwork, results::Results, ts::Int)
    # the Norton current of the OneDOneQ generator is computed from the voltage at ts + 1,
    # so the first network solution of the step starts from the voltages at ts
    for state in results.bus
        state.Vr[ts+1] = state.Vr[ts]
        state.Vi[ts+1] = state.Vi[ts]
    end
    return nothing
end

function bus_voltages_at(network::PhasorNetwork, results::Results, ts::Int)
    V = zeros(ComplexF64, num_buses(results))
    for (bus_idx, state) in enumerate(results.bus)
        V[bus_idx] = bus_voltage_phasor(network, state, ts)
    end
    return V
end

function apply_perturbations!(network::Network, inputs::Inputs, results::Results, ts::Int)
    current_perturbations = perturbations_at(inputs, ts)
    if isempty(current_perturbations)
        return nothing
    end
    # the perturbations only change the state of the elements, and the network is rebuilt from that state
    for perturbation in current_perturbations
        apply_perturbation!(inputs, perturbation)
    end
    rebuild_network!(network, inputs)

    # a perturbation at ts changes the voltages and electrical powers at that instant, so the network is
    # solved again at ts and the step to ts + 1 starts from the values after the perturbation.
    # The states are continuous and keep their values at ts.
    # The Norton current of the OneDOneQ generator depends on V when Xd' differs from Xq', so the network
    # is solved until the voltages stop changing
    for iteration in 1:max_iterations(inputs)
        V_previous = bus_voltages_at(network, results, ts)
        solve_network!(network, inputs, results, ts)
        V = bus_voltages_at(network, results, ts)
        if maximum(abs.(V .- V_previous)) < tolerance(inputs)
            return nothing
        end
    end
    error("The network did not converge after the perturbations at time step $ts.")
end

# advances the simulation from the time index ts to ts + 1
function step!(inputs::Inputs, results::Results, ts::Int)
    network = get_network(inputs)

    apply_perturbations!(network, inputs, results, ts) # rebuild the network and solve it again at ts if there are perturbations at this instant

    predict_voltages!(network, results, ts)
    for (generator, state) in zip(inputs.generators, results.generator)
        predict_generator!(inputs, generator, state, ts)
    end
    for (bus, bus_state) in zip(inputs.buses, results.bus)
        freq_estimation = bus_freq_estimation(bus)
        predict_bus_freq_estimation!(inputs, freq_estimation, bus_state.freq_estimation, bus_state, ts)
    end
    for (ibr, state) in zip(inputs.ibrs, results.ibr)
        bus_idx = ibr_bus(ibr)
        freq_estimation_state = results.bus[bus_idx].freq_estimation
        predict_ibr!(inputs, ibr, state, freq_estimation_state, ts)
    end
    solve_network!(network, inputs, results, ts + 1) # V[ts+1] and Pele[ts+1] from the predicted states

    for iteration in 1:max_iterations(inputs)
        previous_values = convergence_values(results, ts + 1) # values at ts + 1 from the previous iteration

        for (generator, state) in zip(inputs.generators, results.generator)
            step_generator!(inputs, generator, state, ts)
        end
        for (bus, bus_state) in zip(inputs.buses, results.bus)
            freq_estimation = bus_freq_estimation(bus)
            step_bus_freq_estimation!(inputs, freq_estimation, bus_state.freq_estimation, bus_state, ts)
        end
        # the IBR controls take omega at ts + 1 from the frequency estimation of their bus, so they come after it
        for (ibr, state) in zip(inputs.ibrs, results.ibr)
            bus_idx = ibr_bus(ibr)
            freq_estimation_state = results.bus[bus_idx].freq_estimation
            step_ibr!(inputs, ibr, state, freq_estimation_state, ts)
        end
        solve_network!(network, inputs, results, ts + 1)

        # a value not yet computed is NaN, and so is its difference, which keeps the step from converging
        current_values = convergence_values(results, ts + 1)
        differences = abs.(current_values .- previous_values)
        if maximum(differences) < tolerance(inputs)
            return nothing
        end
    end
    error("Time step $ts did not converge after $(max_iterations(inputs)) iterations.")
end

function solve(inputs::Inputs; output_path::String = joinpath(case_path(inputs), "results"))
    results = initialization(inputs)

    for ts in 1:num_time_steps(inputs)
        step!(inputs, results, ts)
    end

    write_results(inputs, results, output_path)

    return results
end
