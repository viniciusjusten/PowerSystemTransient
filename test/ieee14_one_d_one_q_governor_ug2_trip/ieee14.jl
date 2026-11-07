@testset "IEEE 14 with one d one q generators, governors and UG2 trip" begin
    trip_time = 1.0
    inputs = ieee14_inputs(
        @__DIR__;
        perturbations = ieee14_ug2_trip_perturbations(),
        generator_model = "one_d_one_q",
        gov = true,
    )
    results = PST.solve(inputs)

    # every trajectory matches the one stored in the reference file
    test_trajectories(inputs, results, joinpath(@__DIR__, "reference", "trajectories.csv"))

    # before the trip the system stays at its initial operating point, since the reference power of each governor
    # is initialized to hold the mechanical power at the power flow solution
    test_initial_operating_point_kept(inputs, results, trip_time)

    # UG1 and UG2 have governors, whose reference power is a setpoint, so it stays constant during the whole simulation
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

    # after the trip the governor of UG1 moves its mechanical power against the speed deviation, as the droop requires
    ug1 = results.generator[1]
    @test PST.has_governor(PST.get_generator(inputs, 1))
    @test ug1.Pmec[end] != ug1.Pmec[1]
    @test (ug1.Pmec[end] - ug1.Pmec[1]) * (ug1.omega[end] - 1.0) < 0.0
end
