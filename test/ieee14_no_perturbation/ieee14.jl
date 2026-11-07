@testset "IEEE 14 without perturbation" begin
    inputs = ieee14_inputs(@__DIR__)
    results = PST.solve(inputs)

    # without a perturbation the system stays at its initial operating point, so every time step
    # of every trajectory is compared with the power flow solution written in the case

    # the network solved with the initialized machines reproduces the power flow
    for (bus, state) in zip(inputs.buses, results.bus)
        V = state.Vr .+ im .* state.Vi
        voltage_errors = abs.(abs.(V) .- PST.bus_voltage(bus))
        angles = rad2deg.(angle.(V))
        angle_errors = abs.(angles .- rad2deg(PST.bus_angle(bus)))
        @test maximum(voltage_errors) < 1e-6
        @test maximum(angle_errors) < 1e-5
    end

    # rotor angles, in degrees, from the PowerDynamics.jl simulation of the same case,
    # in the order of the generators of the case
    delta0_powerdynamics = [
        20.942285984420202,
        1.4641342940804398,
        -15.68089487965635,
        -18.92899192122492,
        -16.520651677707935,
    ]
    for (gen_idx, generator) in enumerate(inputs.generators)
        state = results.generator[gen_idx]
        P0 = PST.generator_P0(generator)
        Pele_errors = abs.(state.Pele .- P0) .* PST.Sb(inputs)
        Pmec_errors = abs.(state.Pmec .- P0) .* PST.Sb(inputs)
        delta_errors = abs.(rad2deg.(state.delta) .- delta0_powerdynamics[gen_idx])
        omega_errors = abs.(state.omega .- 1.0)
        @test maximum(Pele_errors) < 1e-4
        @test maximum(Pmec_errors) < 1e-4
        @test maximum(delta_errors) < 1e-5
        @test maximum(omega_errors) < 1e-8
    end
end
