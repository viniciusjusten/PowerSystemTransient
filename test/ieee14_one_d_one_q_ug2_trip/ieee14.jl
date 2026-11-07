@testset "IEEE 14 with one d one q generators and UG2 trip" begin
    trip_time = 1.0
    ug2_idx = 2
    inputs = ieee14_inputs(@__DIR__; perturbations = ieee14_ug2_trip_perturbations(), generator_model = "one_d_one_q")
    results = PST.solve(inputs)

    # every trajectory matches the one stored in the reference file
    test_trajectories(inputs, results, joinpath(@__DIR__, "reference", "trajectories.csv"))

    # before the trip the system stays at its initial operating point
    test_initial_operating_point_kept(inputs, results, trip_time)

    time = PST.time_values(inputs)
    # half a step of margin so that the comparisons do not depend on the rounding of ts * integration_step
    margin = PST.integration_step(inputs) / 2
    after_trip = time .> trip_time - margin

    # from the trip on UG2 carries no current and delivers no power, so its mechanical power accelerates the rotor
    ug2 = results.generator[ug2_idx]
    @test PST.generator_tripped(PST.get_generator(inputs, ug2_idx))
    @test all(ug2.id[after_trip] .== 0.0)
    @test all(ug2.iq[after_trip] .== 0.0)
    @test all(ug2.Pele[after_trip] .== 0.0)
    @test ug2.omega[end] > 1.0
end
