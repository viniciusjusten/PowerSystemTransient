@testset "IEEE 14 with one d one q generators and short circuit at bus 14" begin
    fault_time = 1.0
    clearing_time = 1.2
    fault_bus_idx = 14
    opened_circuit_idx = 17
    inputs = ieee14_inputs(
        @__DIR__;
        perturbations = ieee14_short_circuit_perturbations(),
        generator_model = "one_d_one_q",
    )
    results = PST.solve(inputs)

    # every trajectory matches the one stored in the reference file
    test_trajectories(inputs, results, joinpath(@__DIR__, "reference", "trajectories.csv"))

    # before the short circuit the system stays at its initial operating point
    test_initial_operating_point_kept(inputs, results, fault_time)

    time = PST.time_values(inputs)
    # half a step of margin so that the comparisons do not depend on the rounding of ts * integration_step
    margin = PST.integration_step(inputs) / 2
    during_fault = (time .> fault_time - margin) .& (time .< clearing_time - margin)

    # while the short circuit lasts the voltage of the faulted bus is close to zero
    fault_bus = results.bus[fault_bus_idx]
    fault_voltage = abs.(fault_bus.Vr[during_fault] .+ im .* fault_bus.Vi[during_fault])
    @test maximum(fault_voltage) < 1e-3

    # the excitation voltage of every generator stays constant, since there is no voltage regulator
    for state in results.generator
        @test all(state.efd .== state.efd[1])
    end

    # at the end the fault is removed and the line LT13-14 is open
    @test PST.bus_fault_admittance(PST.get_bus(inputs, fault_bus_idx)) == 0.0 + 0.0im
    @test !PST.circuit_in_service(PST.get_circuit(inputs, opened_circuit_idx))
end
