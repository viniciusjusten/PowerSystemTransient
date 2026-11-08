@testset "IEEE 14 with the dynamic phasor network, one d one q generators, governors and voltage regulators, where tripping the IBR that replaces UG2 matches tripping UG2" begin
    # UG2 is a synchronous generator in one case and an IBR in the other, and each case trips it at t = 1 s
    synchronous_inputs = ieee14_inputs(
        joinpath(@__DIR__, "ug2_trip");
        perturbations = ieee14_ug2_trip_perturbations(),
        generator_model = "one_d_one_q",
        gov = true,
        avr = true,
        network = PST.NetworkDP(),
        integration_step = 0.001,
        simulation_horizon = 3.0,
    )
    synchronous_results = PST.solve(synchronous_inputs)

    ibr_inputs = ieee14_inputs(
        joinpath(@__DIR__, "ibr2_trip");
        perturbations = PST.Perturbation[PST.IBRTrip(; name = "IBR2 trip", ibr_idx = 1, time = 1.0)],
        generator_model = "one_d_one_q",
        gov = true,
        avr = true,
        ug2_ibr = true,
        network = PST.NetworkDP(),
        integration_step = 0.001,
        simulation_horizon = 3.0,
    )
    ibr_results = PST.solve(ibr_inputs)

    # Both cases start from the same power flow and, after the trip, have the same network with UG1, UG3, UG6 and
    # UG8, so every trajectory they have in common is the same. The trajectories of UG2 and of IBR2, and the frequency
    # estimation of bus 2, which only the IBR needs, exist in one case only and are not compared
    synchronous_names, synchronous_trajectories = model_trajectories(synchronous_inputs, synchronous_results)
    ibr_names, ibr_trajectories = model_trajectories(ibr_inputs, ibr_results)
    compared_names = String[]
    for (trajectory_name, ibr_trajectory) in zip(ibr_names, ibr_trajectories)
        synchronous_idx = findfirst(==(trajectory_name), synchronous_names)
        if synchronous_idx === nothing
            continue
        end
        push!(compared_names, trajectory_name)
        synchronous_trajectory = synchronous_trajectories[synchronous_idx]
        @testset "$trajectory_name" begin
            max_difference = maximum(abs.(ibr_trajectory .- synchronous_trajectory))
            @test max_difference < 1e-6
        end
    end

    # the comparison covers the machines left in service
    for generator_name in ("UG1", "UG3", "UG6", "UG8")
        @test "generator_$(generator_name)_omega" in compared_names
    end
end
