@testset "IEEE 14 with one d one q generators, governors, voltage regulators, bus frequency estimations and UG2 trip" begin
    inputs = ieee14_inputs(
        @__DIR__;
        perturbations = ieee14_ug2_trip_perturbations(),
        generator_model = "one_d_one_q",
        gov = true,
        avr = true,
    )

    # the buses of the machines left in service, UG1, UG3, UG6 and UG8, estimate their frequency
    for gen_idx in (1, 3, 4, 5)
        generator = PST.get_generator(inputs, gen_idx)
        bus_idx = PST.generator_bus(generator)
        bus = PST.get_bus(inputs, bus_idx)
        estimation_name = "$(PST.bus_name(bus)) freq"
        bus.freq_estimation = PST.FilteredAngleDerivative(; name = estimation_name, Tf = 0.05, Tw = 0.1)
    end

    results = PST.solve(inputs)

    # The estimated frequency of each bus follows the speed of the machine connected to it. The speeds deviate by up
    # to 1e-2 pu after the trip, and the largest gap, about 2.4e-3 pu, happens in the first swing, when the filters
    # lag the fast change of the speed by about Tf + Tw
    for gen_idx in (1, 3, 4, 5)
        generator = PST.get_generator(inputs, gen_idx)
        state = results.generator[gen_idx]
        bus_idx = PST.generator_bus(generator)
        estimation_state = results.bus[bus_idx].freq_estimation
        speed_difference = estimation_state.omega .- state.omega
        @test maximum(abs, speed_difference) < 3e-3
    end
end
