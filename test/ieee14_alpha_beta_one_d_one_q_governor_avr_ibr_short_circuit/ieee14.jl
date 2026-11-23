@testset "IEEE 14 with the alpha beta network, one d one q generators, governors, voltage regulators, UG2 replaced by an IBR and short circuit at bus 14" begin
    fault_time = 0.02
    clearing_time = 0.07
    inputs = ieee14_inputs(
        @__DIR__;
        perturbations = ieee14_short_circuit_perturbations(; fault_time = fault_time, clearing_time = clearing_time),
        generator_model = "one_d_one_q",
        gov = true,
        avr = true,
        ug2_ibr = true,
        network = PST.NetworkAlphaBeta(),
        integration_step = 5e-5,
        simulation_horizon = 0.1,
    )
    results = PST.solve(inputs)

    # every trajectory, including those of the IBR, matches the one stored in the reference file
    test_trajectories(inputs, results, joinpath(@__DIR__, "reference", "trajectories.csv"))

    # before the short circuit the system stays close to its initial operating point, within the tolerance of the
    # alpha beta network
    test_initial_operating_point_kept(inputs, results, fault_time; tolerance = alpha_beta_operating_point_tolerance())
end
