@testset "IEEE 14 with one d one q generators, governors, voltage regulators and UG2 trip" begin
    trip_time = 1.0
    inputs = ieee14_inputs(
        @__DIR__;
        perturbations = ieee14_ug2_trip_perturbations(),
        generator_model = "one_d_one_q",
        gov = true,
        avr = true,
    )
    results = PST.solve(inputs)

    # every trajectory matches the one stored in the reference file
    test_trajectories(inputs, results, joinpath(@__DIR__, "reference", "trajectories.csv"))

    # before the trip the system stays at its initial operating point, since the reference power of each governor
    # and the reference voltage of each regulator are initialized at the power flow solution
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

    # Every machine has a voltage regulator, including the synchronous condensers UG3, UG6 and UG8. The reference
    # voltage is a setpoint, so it stays constant, and the field voltage is the limited output of the lag at every instant
    for (generator, state) in zip(inputs.generators, results.generator)
        @test PST.has_voltage_regulator(generator)
        voltage_regulator = PST.generator_voltage_regulator(generator)
        Lmn = PST.voltage_regulator_Lmn(voltage_regulator)
        Lmx = PST.voltage_regulator_Lmx(voltage_regulator)
        voltage_regulator_state = state.voltage_regulator
        @test all(voltage_regulator_state.Vref .== voltage_regulator_state.Vref[1])
        limited_efdu = clamp.(voltage_regulator_state.efdu, Lmn, Lmx)
        @test all(state.efd .== limited_efdu)
    end

    # UG2 supplied reactive power before the trip, so the terminal voltages of the machines left in service drop and
    # their regulators end with a higher field voltage to make up for it
    for gen_idx in (1, 3, 4, 5)
        state = results.generator[gen_idx]
        @test minimum(state.Vt) < state.Vt[1]
        @test state.efd[end] > state.efd[1]
    end
end
