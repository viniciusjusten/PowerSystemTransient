@testset "IEEE 14 with one d one q generators, governors, voltage regulators, UG2 replaced by an IBR and short circuit at bus 14" begin
    fault_time = 1.0
    inputs = ieee14_inputs(
        @__DIR__;
        perturbations = ieee14_short_circuit_perturbations(),
        generator_model = "one_d_one_q",
        gov = true,
        avr = true,
        ug2_ibr = true,
    )
    results = PST.solve(inputs)

    # every trajectory, including those of the IBR, matches the one stored in the reference file
    test_trajectories(inputs, results, joinpath(@__DIR__, "reference", "trajectories.csv"))

    # before the short circuit the system stays at its initial operating point, since the setpoints of the IBR come
    # from the network solution at t = 0
    test_initial_operating_point_kept(inputs, results, fault_time)
end
