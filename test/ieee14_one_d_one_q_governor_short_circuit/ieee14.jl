@testset "IEEE 14 with one d one q generators, governors and short circuit at bus 14" begin
    fault_time = 1.0
    clearing_time = 1.2
    fault_bus_idx = 14
    opened_circuit_idx = 17
    inputs = ieee14_inputs(
        @__DIR__;
        perturbations = ieee14_short_circuit_perturbations(),
        generator_model = "one_d_one_q",
        gov = true,
    )
    results = PST.solve(inputs)

    # every trajectory matches the one stored in the reference file
    test_trajectories(inputs, results, joinpath(@__DIR__, "reference", "trajectories.csv"))

    # before the short circuit the system stays at its initial operating point, since the reference power of each
    # governor is initialized to hold the mechanical power at the power flow solution
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
        @test isnothing(state.voltage_regulator)
    end

    # UG1 and UG2 have governors, whose reference power is a setpoint, so it stays constant
    for gen_idx in (1, 2)
        governor_state = results.generator[gen_idx].governor
        @test all(governor_state.Pref .== governor_state.Pref[1])
    end

    # UG3, UG6 and UG8 are synchronous condensers without governor, so their mechanical power stays constant
    for gen_idx in (3, 4, 5)
        generator = PST.get_generator(inputs, gen_idx)
        state = results.generator[gen_idx]
        @test !PST.has_governor(generator)
        @test isnothing(state.governor)
        @test all(state.Pmec .== state.Pmec[1])
    end

    # The governor of UG1 moves the valve against the speed deviation, as the droop requires. At the highest speed
    # after the fault the valve is below its initial position, and at the lowest speed it is above it
    ug1 = results.generator[1]
    ug1_governor = ug1.governor
    @test PST.has_governor(PST.get_generator(inputs, 1))
    highest_speed_idx = argmax(ug1.omega)
    lowest_speed_idx = argmin(ug1.omega)
    @test ug1.omega[highest_speed_idx] > 1.0
    @test ug1_governor.Pmec_1[highest_speed_idx] < ug1_governor.Pmec_1[1]
    @test ug1.omega[lowest_speed_idx] < 1.0
    @test ug1_governor.Pmec_1[lowest_speed_idx] > ug1_governor.Pmec_1[1]

    # at the end the fault is removed and the line LT13-14 is open
    @test PST.bus_fault_admittance(PST.get_bus(inputs, fault_bus_idx)) == 0.0 + 0.0im
    @test !PST.circuit_in_service(PST.get_circuit(inputs, opened_circuit_idx))
end
