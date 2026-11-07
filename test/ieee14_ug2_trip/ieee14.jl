@testset "IEEE 14 with UG2 trip" begin
    trip_time = 1.0
    ug2_idx = 2
    perturbations =
        PST.Perturbation[PST.GeneratorTrip(; name = "UG2 trip", generator_idx = ug2_idx, time = trip_time)]
    inputs = ieee14_inputs(@__DIR__; perturbations = perturbations)
    results = PST.solve(inputs)

    # every trajectory matches the one stored in the reference file
    test_trajectories(inputs, results, joinpath(@__DIR__, "reference", "trajectories.csv"))

    time = PST.time_values(inputs)
    # half a step of margin so that the comparisons do not depend on the rounding of ts * integration_step
    margin = PST.integration_step(inputs) / 2
    before_trip = time .< trip_time - margin
    after_trip = time .> trip_time - margin

    # before the trip the system stays at its initial operating point
    for state in results.generator
        @test maximum(abs.(state.delta[before_trip] .- state.delta[1])) < 1e-8
        @test maximum(abs.(state.omega[before_trip] .- 1.0)) < 1e-8
    end
    for state in results.bus
        @test maximum(abs.(state.Vr[before_trip] .- state.Vr[1])) < 1e-8
        @test maximum(abs.(state.Vi[before_trip] .- state.Vi[1])) < 1e-8
    end

    # from the trip on UG2 delivers no power, so its mechanical power accelerates the rotor
    ug2 = results.generator[ug2_idx]
    @test PST.generator_tripped(PST.get_generator(inputs, ug2_idx))
    @test all(ug2.Pele[after_trip] .== 0.0)
    @test ug2.omega[end] > 1.0

    # the initialization undoes the trip, so a second simulation of the same inputs gives the same results
    rerun_results = PST.solve(inputs)
    for gen_idx in 1:PST.num_generators(inputs)
        @test rerun_results.generator[gen_idx].delta == results.generator[gen_idx].delta
    end
    for bus_idx in 1:PST.num_buses(inputs)
        @test rerun_results.bus[bus_idx].Vr == results.bus[bus_idx].Vr
    end
end
