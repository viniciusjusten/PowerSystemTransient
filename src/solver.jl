# values at ts + 1 from the previous corrector iteration, used to check convergence
mutable struct StepCache
    delta::Vector{Float64}
    omega::Vector{Float64}
    eq_prime::Vector{Float64}
    ed_prime::Vector{Float64}
    efd::Vector{Float64}
    Pmec_1::Vector{Float64}
    Pmec::Vector{Float64}
    Vr::Vector{Float64}
    Vi::Vector{Float64}
    x_theta::Vector{Float64}
    omega_estim::Vector{Float64}
    yp::Vector{Float64}
    Pref::Vector{Float64}
    yq::Vector{Float64}
    Qref::Vector{Float64}
    Ip::Vector{Float64}
    Iq::Vector{Float64}
end

function StepCache(inputs::Inputs)
    n_generators = num_generators(inputs)
    n_buses = num_buses(inputs)
    n_ibrs = num_ibrs(inputs)
    return StepCache(
        zeros(n_generators), zeros(n_generators), zeros(n_generators), zeros(n_generators), zeros(n_generators), zeros(n_generators), zeros(n_generators),
        zeros(n_buses), zeros(n_buses), zeros(n_buses), zeros(n_buses),
        zeros(n_ibrs), zeros(n_ibrs), zeros(n_ibrs), zeros(n_ibrs), zeros(n_ibrs), zeros(n_ibrs),
    )
end

function cache_step!(inputs::Inputs, results::Results, ts::Int, caches::StepCache)
    for gen_idx in 1:num_generators(inputs)
        caches.delta[gen_idx] = results.generator[gen_idx].delta[ts+1]
        caches.omega[gen_idx] = results.generator[gen_idx].omega[ts+1]
        if results.generator[gen_idx] isa ClassicGeneratorState
            caches.eq_prime[gen_idx] = 0.0
            caches.ed_prime[gen_idx] = 0.0
            caches.efd[gen_idx] = 0.0
        elseif results.generator[gen_idx] isa OneDOneQGeneratorState
            caches.eq_prime[gen_idx] = results.generator[gen_idx].eq_prime[ts+1]
            caches.ed_prime[gen_idx] = results.generator[gen_idx].ed_prime[ts+1]
            caches.efd[gen_idx] = results.generator[gen_idx].efd[ts+1]
        else
            error("Unknown generator state type: $(typeof(results.generator[gen_idx]))")
        end
        caches.Pmec[gen_idx] = results.generator[gen_idx].Pmec[ts+1]
        # Pmec_1 belongs to the governor, so only the generators with a governor have it
        governor_state = results.generator[gen_idx].governor
        if isnothing(governor_state)
            caches.Pmec_1[gen_idx] = 0.0
        else
            caches.Pmec_1[gen_idx] = governor_state.Pmec_1[ts+1]
        end
    end
    for bus_idx in 1:num_buses(inputs)
        caches.Vr[bus_idx] = results.bus[bus_idx].Vr[ts+1]
        caches.Vi[bus_idx] = results.bus[bus_idx].Vi[ts+1]
        # x_theta and the estimated omega belong to the frequency estimation, so only the buses with an estimation
        # have them
        freq_estimation_state = results.bus[bus_idx].freq_estimation
        if isnothing(freq_estimation_state)
            caches.x_theta[bus_idx] = 0.0
            caches.omega_estim[bus_idx] = 0.0
        else
            caches.x_theta[bus_idx] = freq_estimation_state.x_theta[ts+1]
            caches.omega_estim[bus_idx] = freq_estimation_state.omega[ts+1]
        end
    end
    for ibr_idx in 1:num_ibrs(inputs)
        state = results.ibr[ibr_idx]
        caches.yp[ibr_idx] = state.yp[ts+1]
        caches.Pref[ibr_idx] = state.Pref[ts+1]
        caches.yq[ibr_idx] = state.yq[ts+1]
        caches.Qref[ibr_idx] = state.Qref[ts+1]
        caches.Ip[ibr_idx] = state.Ip[ts+1]
        caches.Iq[ibr_idx] = state.Iq[ts+1]
    end
    return nothing
end

function step_converged(inputs::Inputs, results::Results, ts::Int, caches::StepCache)
    max_dif = 0.0

    # the first index in results corresponds to the initial state (t = 0),
    # so the current time step is ts + 1
    for gen_idx in 1:num_generators(inputs)
        max_dif = max(max_dif, abs(caches.delta[gen_idx] - results.generator[gen_idx].delta[ts+1]))
        max_dif = max(max_dif, abs(caches.omega[gen_idx] - results.generator[gen_idx].omega[ts+1]))
        if results.generator[gen_idx] isa ClassicGeneratorState
            continue
        elseif results.generator[gen_idx] isa OneDOneQGeneratorState
            max_dif = max(max_dif, abs(caches.eq_prime[gen_idx] - results.generator[gen_idx].eq_prime[ts+1]))
            max_dif = max(max_dif, abs(caches.ed_prime[gen_idx] - results.generator[gen_idx].ed_prime[ts+1]))
            max_dif = max(max_dif, abs(caches.efd[gen_idx] - results.generator[gen_idx].efd[ts+1]))
        else
            error("Unknown generator state type: $(typeof(results.generator[gen_idx]))")
        end
        governor_state = results.generator[gen_idx].governor
        if !isnothing(governor_state)
            max_dif = max(max_dif, abs(caches.Pmec_1[gen_idx] - governor_state.Pmec_1[ts+1]))
        end
        max_dif = max(max_dif, abs(caches.Pmec[gen_idx] - results.generator[gen_idx].Pmec[ts+1]))
    end
    for bus_idx in 1:num_buses(inputs)
        max_dif = max(max_dif, abs(caches.Vr[bus_idx] - results.bus[bus_idx].Vr[ts+1]))
        max_dif = max(max_dif, abs(caches.Vi[bus_idx] - results.bus[bus_idx].Vi[ts+1]))
        freq_estimation_state = results.bus[bus_idx].freq_estimation
        if !isnothing(freq_estimation_state)
            max_dif = max(max_dif, abs(caches.x_theta[bus_idx] - freq_estimation_state.x_theta[ts+1]))
            max_dif = max(max_dif, abs(caches.omega_estim[bus_idx] - freq_estimation_state.omega[ts+1]))
        end
    end
    for ibr_idx in 1:num_ibrs(inputs)
        state = results.ibr[ibr_idx]
        max_dif = max(max_dif, abs(caches.yp[ibr_idx] - state.yp[ts+1]))
        max_dif = max(max_dif, abs(caches.Pref[ibr_idx] - state.Pref[ts+1]))
        max_dif = max(max_dif, abs(caches.yq[ibr_idx] - state.yq[ts+1]))
        max_dif = max(max_dif, abs(caches.Qref[ibr_idx] - state.Qref[ts+1]))
        max_dif = max(max_dif, abs(caches.Ip[ibr_idx] - state.Ip[ts+1]))
        max_dif = max(max_dif, abs(caches.Iq[ibr_idx] - state.Iq[ts+1]))
    end

    return max_dif < tolerance(inputs)
end

function predict_voltages!(results::Results, ts::Int)
    # the Norton current of the OneDOneQ generator is computed from the voltage at ts + 1,
    # so the first network solution of the step starts from the voltages at ts
    for state in results.bus
        state.Vr[ts+1] = state.Vr[ts]
        state.Vi[ts+1] = state.Vi[ts]
    end
    return nothing
end

function bus_voltages_at(results::Results, ts::Int)
    V = zeros(ComplexF64, num_buses(results))
    for (bus_idx, state) in enumerate(results.bus)
        V[bus_idx] = state.Vr[ts] + im * state.Vi[ts]
    end
    return V
end

function apply_perturbations!(inputs::Inputs, results::Results, ts::Int)
    # a perturbation at ts changes the voltages and electrical powers at that instant, so the network is
    # solved again at ts and the step to ts + 1 starts from the values after the perturbation.
    # The states are continuous and keep their values at ts
    if !cache_Ybus!(inputs, ts)
        return nothing
    end
    # the Norton current of the OneDOneQ generator depends on V when Xd' differs from Xq', so the network
    # is solved until the voltages stop changing
    for iteration in 1:max_iterations(inputs)
        V_previous = bus_voltages_at(results, ts)
        solve_network!(inputs, results, ts)
        V = bus_voltages_at(results, ts)
        if maximum(abs.(V .- V_previous)) < tolerance(inputs)
            return nothing
        end
    end
    error("The network did not converge after the perturbations at time step $ts.")
end

# advances the simulation from the time index ts to ts + 1
function step!(inputs::Inputs, results::Results, ts::Int)
    caches = StepCache(inputs)

    apply_perturbations!(inputs, results, ts) # update Ybus and the network solution at ts if there are perturbations at this instant

    predict_voltages!(results, ts)
    for (generator, state) in zip(inputs.generators, results.generator)
        governor = generator_governor(generator)
        voltage_regulator = generator_voltage_regulator(generator)
        predict_omega!(inputs, generator, state, ts)
        predict_delta!(inputs, state, ts)
        predict_Pmec_1!(inputs, generator, governor, state.governor, state, ts)
        predict_Pmec!(inputs, governor, state.governor, state, ts)
        if generator isa OneDOneQGenerator
            predict_ed_prime!(inputs, generator, state, ts)
            predict_eq_prime!(inputs, generator, state, ts)
        end
        predict_efd!(inputs, voltage_regulator, state.voltage_regulator, state, ts)
    end
    for (bus, bus_state) in zip(inputs.buses, results.bus)
        freq_estimation = bus_freq_estimation(bus)
        predict_bus_freq_estimation!(inputs, freq_estimation, bus_state.freq_estimation, bus_state, ts)
    end
    for (ibr, state) in zip(inputs.ibrs, results.ibr)
        bus_idx = ibr_bus(ibr)
        freq_estimation_state = results.bus[bus_idx].freq_estimation
        predict_active_power_control!(inputs, ibr, state, freq_estimation_state, ts)
        predict_reactive_power_control!(inputs, ibr, state, ts)
        predict_converter!(inputs, ibr, state, ts)
    end
    solve_network!(inputs, results, ts + 1) # V[ts+1] and Pele[ts+1] from the predicted states

    for iteration in 1:max_iterations(inputs)
        cache_step!(inputs, results, ts, caches) # delta, omega, Vr and Vi at ts+1

        for (generator, state) in zip(inputs.generators, results.generator)
            governor = generator_governor(generator)
            voltage_regulator = generator_voltage_regulator(generator)
            step_omega!(inputs, generator, state, ts)
            step_delta!(inputs, state, ts)
            step_Pmec_1!(inputs, generator, governor, state.governor, state, ts)
            step_Pmec!(inputs, governor, state.governor, state, ts)
            if generator isa OneDOneQGenerator
                step_ed_prime!(inputs, generator, state, ts)
                step_eq_prime!(inputs, generator, state, ts)
            end
            step_efd!(inputs, voltage_regulator, state.voltage_regulator, state, ts)
        end
        for (bus, bus_state) in zip(inputs.buses, results.bus)
            freq_estimation = bus_freq_estimation(bus)
            step_bus_freq_estimation!(inputs, freq_estimation, bus_state.freq_estimation, bus_state, ts)
        end
        # the IBR controls take omega at ts + 1 from the frequency estimation of their bus, so they come after it, and
        # the converter takes Pref and Qref at ts + 1 from the controls
        for (ibr, state) in zip(inputs.ibrs, results.ibr)
            bus_idx = ibr_bus(ibr)
            freq_estimation_state = results.bus[bus_idx].freq_estimation
            step_active_power_control!(inputs, ibr, state, freq_estimation_state, ts)
            step_reactive_power_control!(inputs, ibr, state, ts)
            step_converter!(inputs, ibr, state, ts)
        end
        solve_network!(inputs, results, ts + 1)

        if step_converged(inputs, results, ts, caches)
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
