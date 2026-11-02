# The IEEE 14 case, with every element written explicitly. The powers and the line parameters are in pu of the
# 100 MVA system base, and the generator parameters are in pu of the machine base. The bus voltages and the
# generator powers are the power flow solution of the case

function ieee14_buses()
    return nsLFT.Bus[
        nsLFT.Bus(; name = "Bus 01", voltage = 1.0, angle = deg2rad(0.0), shunt_susceptance = 0.0, generator_idx = 1),
        nsLFT.Bus(;
            name = "Bus 02",
            voltage = 1.0,
            angle = deg2rad(-6.3985139),
            shunt_susceptance = 0.0,
            generator_idx = 2,
            load_idx = 1,
        ),
        nsLFT.Bus(;
            name = "Bus 03",
            voltage = 1.0,
            angle = deg2rad(-15.680895),
            shunt_susceptance = 0.0,
            generator_idx = 3,
            load_idx = 2,
        ),
        nsLFT.Bus(;
            name = "Bus 04",
            voltage = 0.96359158,
            angle = deg2rad(-12.621613),
            shunt_susceptance = 0.0,
            load_idx = 3,
        ),
        nsLFT.Bus(;
            name = "Bus 05",
            voltage = 0.96230396,
            angle = deg2rad(-10.908482),
            shunt_susceptance = 0.0,
            load_idx = 4,
        ),
        nsLFT.Bus(;
            name = "Bus 06",
            voltage = 1.0,
            angle = deg2rad(-18.928992),
            shunt_susceptance = 0.0,
            generator_idx = 4,
            load_idx = 5,
        ),
        nsLFT.Bus(; name = "Bus 07", voltage = 0.98848533, angle = deg2rad(-16.520652), shunt_susceptance = 0.0),
        nsLFT.Bus(;
            name = "Bus 08",
            voltage = 1.0,
            angle = deg2rad(-16.520652),
            shunt_susceptance = 0.0,
            generator_idx = 5,
        ),
        # capacitor bank of 19 Mvar at 1 pu
        nsLFT.Bus(;
            name = "Bus 09",
            voltage = 0.9848161,
            angle = deg2rad(-18.57159),
            shunt_susceptance = 0.19,
            load_idx = 6,
        ),
        nsLFT.Bus(;
            name = "Bus 10",
            voltage = 0.97971895,
            angle = deg2rad(-18.961085),
            shunt_susceptance = 0.0,
            load_idx = 7,
        ),
        nsLFT.Bus(;
            name = "Bus 11",
            voltage = 0.98622695,
            angle = deg2rad(-19.087288),
            shunt_susceptance = 0.0,
            load_idx = 8,
        ),
        nsLFT.Bus(;
            name = "Bus 12",
            voltage = 0.98390191,
            angle = deg2rad(-19.828868),
            shunt_susceptance = 0.0,
            load_idx = 9,
        ),
        nsLFT.Bus(;
            name = "Bus 13",
            voltage = 0.9790928,
            angle = deg2rad(-19.83645),
            shunt_susceptance = 0.0,
            load_idx = 10,
        ),
        nsLFT.Bus(;
            name = "Bus 14",
            voltage = 0.96297339,
            angle = deg2rad(-20.274264),
            shunt_susceptance = 0.0,
            load_idx = 11,
        ),
    ]
end

# the shunt susceptance of a line is the total of its pi model
function ieee14_circuits()
    return nsLFT.Circuit[
        nsLFT.Circuit(;
            name = "LT1-2",
            from_bus_idx = 1,
            to_bus_idx = 2,
            resistance = 0.0193,
            reactance = 0.0592,
            shunt_conductance = 0.0,
            shunt_susceptance = 0.0528,
            tap_ratio = 1.0,
        ),
        nsLFT.Circuit(;
            name = "LT1-5",
            from_bus_idx = 1,
            to_bus_idx = 5,
            resistance = 0.05403,
            reactance = 0.223,
            shunt_conductance = 0.0,
            shunt_susceptance = 0.0492,
            tap_ratio = 1.0,
        ),
        nsLFT.Circuit(;
            name = "LT2-3",
            from_bus_idx = 2,
            to_bus_idx = 3,
            resistance = 0.04699,
            reactance = 0.198,
            shunt_conductance = 0.0,
            shunt_susceptance = 0.0438,
            tap_ratio = 1.0,
        ),
        nsLFT.Circuit(;
            name = "LT2-4",
            from_bus_idx = 2,
            to_bus_idx = 4,
            resistance = 0.05811,
            reactance = 0.1763,
            shunt_conductance = 0.0,
            shunt_susceptance = 0.0374,
            tap_ratio = 1.0,
        ),
        nsLFT.Circuit(;
            name = "LT2-5",
            from_bus_idx = 2,
            to_bus_idx = 5,
            resistance = 0.05695,
            reactance = 0.1739,
            shunt_conductance = 0.0,
            shunt_susceptance = 0.034,
            tap_ratio = 1.0,
        ),
        nsLFT.Circuit(;
            name = "LT3-4",
            from_bus_idx = 3,
            to_bus_idx = 4,
            resistance = 0.06701,
            reactance = 0.171,
            shunt_conductance = 0.0,
            shunt_susceptance = 0.0346,
            tap_ratio = 1.0,
        ),
        nsLFT.Circuit(;
            name = "LT4-5",
            from_bus_idx = 4,
            to_bus_idx = 5,
            resistance = 0.01335,
            reactance = 0.0421,
            shunt_conductance = 0.0,
            shunt_susceptance = 0.0128,
            tap_ratio = 1.0,
        ),
        nsLFT.Circuit(;
            name = "LT6-11",
            from_bus_idx = 6,
            to_bus_idx = 11,
            resistance = 0.09498,
            reactance = 0.1989,
            shunt_conductance = 0.0,
            shunt_susceptance = 0.0,
            tap_ratio = 1.0,
        ),
        nsLFT.Circuit(;
            name = "LT6-12",
            from_bus_idx = 6,
            to_bus_idx = 12,
            resistance = 0.12291,
            reactance = 0.2558,
            shunt_conductance = 0.0,
            shunt_susceptance = 0.0,
            tap_ratio = 1.0,
        ),
        nsLFT.Circuit(;
            name = "LT6-13",
            from_bus_idx = 6,
            to_bus_idx = 13,
            resistance = 0.06615,
            reactance = 0.1303,
            shunt_conductance = 0.0,
            shunt_susceptance = 0.0,
            tap_ratio = 1.0,
        ),
        nsLFT.Circuit(;
            name = "LT7-8",
            from_bus_idx = 7,
            to_bus_idx = 8,
            resistance = 0.0,
            reactance = 0.1761,
            shunt_conductance = 0.0,
            shunt_susceptance = 0.0,
            tap_ratio = 1.0,
        ),
        nsLFT.Circuit(;
            name = "LT7-9",
            from_bus_idx = 7,
            to_bus_idx = 9,
            resistance = 0.0,
            reactance = 0.11,
            shunt_conductance = 0.0,
            shunt_susceptance = 0.0,
            tap_ratio = 1.0,
        ),
        nsLFT.Circuit(;
            name = "LT9-10",
            from_bus_idx = 9,
            to_bus_idx = 10,
            resistance = 0.03181,
            reactance = 0.0845,
            shunt_conductance = 0.0,
            shunt_susceptance = 0.0,
            tap_ratio = 1.0,
        ),
        nsLFT.Circuit(;
            name = "LT9-14",
            from_bus_idx = 9,
            to_bus_idx = 14,
            resistance = 0.12711,
            reactance = 0.2704,
            shunt_conductance = 0.0,
            shunt_susceptance = 0.0,
            tap_ratio = 1.0,
        ),
        nsLFT.Circuit(;
            name = "LT10-11",
            from_bus_idx = 10,
            to_bus_idx = 11,
            resistance = 0.08205,
            reactance = 0.1921,
            shunt_conductance = 0.0,
            shunt_susceptance = 0.0,
            tap_ratio = 1.0,
        ),
        nsLFT.Circuit(;
            name = "LT12-13",
            from_bus_idx = 12,
            to_bus_idx = 13,
            resistance = 0.22092,
            reactance = 0.1999,
            shunt_conductance = 0.0,
            shunt_susceptance = 0.0,
            tap_ratio = 1.0,
        ),
        nsLFT.Circuit(;
            name = "LT13-14",
            from_bus_idx = 13,
            to_bus_idx = 14,
            resistance = 0.17093,
            reactance = 0.348,
            shunt_conductance = 0.0,
            shunt_susceptance = 0.0,
            tap_ratio = 1.0,
        ),
        # The tap a of the transformers sits on the from side, so the series current is (V_from / a - V_to) / Z,
        # which in the admittance matrix of nsLFT means tap_ratio = 1 / a
        nsLFT.Circuit(;
            name = "TR4-7",
            from_bus_idx = 4,
            to_bus_idx = 7,
            resistance = 0.0,
            reactance = 0.2091,
            shunt_conductance = 0.0,
            shunt_susceptance = 0.0,
            tap_ratio = 1 / 0.978,
        ),
        nsLFT.Circuit(;
            name = "TR4-9",
            from_bus_idx = 4,
            to_bus_idx = 9,
            resistance = 0.0,
            reactance = 0.5562,
            shunt_conductance = 0.0,
            shunt_susceptance = 0.0,
            tap_ratio = 1 / 0.969,
        ),
        nsLFT.Circuit(;
            name = "TR5-6",
            from_bus_idx = 5,
            to_bus_idx = 6,
            resistance = 0.0,
            reactance = 0.252,
            shunt_conductance = 0.0,
            shunt_susceptance = 0.0,
            tap_ratio = 1 / 0.932,
        ),
    ]
end

# Vn is NaN because the case gives no rated voltage and the models do not use it yet
function ieee14_classic_generators()
    return nsLFT.Generator[
        nsLFT.ClassicGenerator(;
            name = "UG1",
            bus = 1,
            Sn = 286.1881227,
            Vn = NaN,
            Xd_prime = 0.4,
            Ra = 0.0,
            H = 5.0,
            D = 2.0,
            P0 = 2.560510642,
            Q0 = -0.46420896712,
        ),
        nsLFT.ClassicGenerator(;
            name = "UG2",
            bus = 2,
            Sn = 88.09334324,
            Vn = NaN,
            Xd_prime = 0.4,
            Ra = 0.0,
            H = 5.0,
            D = 2.0,
            P0 = 0.4,
            Q0 = 0.69417986775,
        ),
        nsLFT.ClassicGenerator(;
            name = "UG3",
            bus = 3,
            Sn = 75.889,
            Vn = NaN,
            Xd_prime = 0.4,
            Ra = 0.0,
            H = 5.0,
            D = 2.0,
            P0 = 0.0,
            Q0 = 0.690129873152,
        ),
        nsLFT.ClassicGenerator(;
            name = "UG6",
            bus = 6,
            Sn = 40.722,
            Vn = NaN,
            Xd_prime = 0.4,
            Ra = 0.0,
            H = 5.0,
            D = 2.0,
            P0 = 0.0,
            Q0 = 0.370347922599,
        ),
        nsLFT.ClassicGenerator(;
            name = "UG8",
            bus = 8,
            Sn = 7.1863,
            Vn = NaN,
            Xd_prime = 0.4,
            Ra = 0.0,
            H = 5.0,
            D = 2.0,
            P0 = 0.0,
            Q0 = 0.0653871095968,
        ),
    ]
end

function ieee14_one_d_one_q_generators()
    return nsLFT.Generator[
        nsLFT.OneDOneQGenerator(;
            name = "UG1",
            bus = 1,
            Sn = 286.1881227,
            Vn = NaN,
            Xd = 1.8,
            Xd_prime = 0.4,
            Xq = 1.7,
            Xq_prime = 0.4,
            Ra = 0.0,
            Td0_prime = 8.0,
            Tq0_prime = 0.4,
            H = 5.0,
            D = 2.0,
            P0 = 2.560510642,
            Q0 = -0.46420896712,
        ),
        nsLFT.OneDOneQGenerator(;
            name = "UG2",
            bus = 2,
            Sn = 88.09334324,
            Vn = NaN,
            Xd = 1.8,
            Xd_prime = 0.4,
            Xq = 1.7,
            Xq_prime = 0.4,
            Ra = 0.0,
            Td0_prime = 8.0,
            Tq0_prime = 0.4,
            H = 5.0,
            D = 2.0,
            P0 = 0.4,
            Q0 = 0.69417986775,
        ),
        nsLFT.OneDOneQGenerator(;
            name = "UG3",
            bus = 3,
            Sn = 75.889,
            Vn = NaN,
            Xd = 1.8,
            Xd_prime = 0.4,
            Xq = 1.7,
            Xq_prime = 0.4,
            Ra = 0.0,
            Td0_prime = 8.0,
            Tq0_prime = 0.4,
            H = 5.0,
            D = 2.0,
            P0 = 0.0,
            Q0 = 0.690129873152,
        ),
        nsLFT.OneDOneQGenerator(;
            name = "UG6",
            bus = 6,
            Sn = 40.722,
            Vn = NaN,
            Xd = 1.8,
            Xd_prime = 0.4,
            Xq = 1.7,
            Xq_prime = 0.4,
            Ra = 0.0,
            Td0_prime = 8.0,
            Tq0_prime = 0.4,
            H = 5.0,
            D = 2.0,
            P0 = 0.0,
            Q0 = 0.370347922599,
        ),
        nsLFT.OneDOneQGenerator(;
            name = "UG8",
            bus = 8,
            Sn = 7.1863,
            Vn = NaN,
            Xd = 1.8,
            Xd_prime = 0.4,
            Xq = 1.7,
            Xq_prime = 0.4,
            Ra = 0.0,
            Td0_prime = 8.0,
            Tq0_prime = 0.4,
            H = 5.0,
            D = 2.0,
            P0 = 0.0,
            Q0 = 0.0653871095968,
        ),
    ]
end

# the generator model is "classic" or "one_d_one_q", and the same model is used for every generator of the case
function ieee14_generators(generator_model::String, gov::Bool, avr::Bool)
    generators = nsLFT.Generator[]
    if generator_model == "classic"
        generators = ieee14_classic_generators()
    elseif generator_model == "one_d_one_q"
        generators = ieee14_one_d_one_q_generators()
    else
        error("Unknown generator model $generator_model.")
    end

    # UG3, UG6 and UG8 are synchronous condensers, which have no turbine and therefore no governor
    if gov
        generators[1].governor =
            nsLFT.Droop1stOrder(; name = "UG1 gov", R = 0.05, Tc = 0.01, Tr = 2.0, omega_ref = 60.0)
        generators[2].governor =
            nsLFT.Droop1stOrder(; name = "UG2 gov", R = 0.05, Tc = 0.01, Tr = 2.0, omega_ref = 60.0)
    end

    # Every machine has a voltage regulator, including the synchronous condensers, whose only role is to hold the
    # voltage of their bus. The classic model keeps E' constant, so it takes no voltage regulator
    if avr
        if generator_model == "classic"
            error("The classic generator model has no voltage regulator.")
        end
        for generator in generators
            regulator_name = "$(nsLFT.generator_name(generator)) avr"
            generator.voltage_regulator =
                nsLFT.Static1stOrder(; name = regulator_name, K = 50.0, T = 0.05, Lmn = -5.0, Lmx = 5.0)
        end
    end
    return generators
end

function ieee14_loads()
    return nsLFT.Load[
        nsLFT.ConstantZLoad(; name = "CR2", bus = 2, P0 = 0.217, Q0 = 0.127),
        nsLFT.ConstantZLoad(; name = "CR3", bus = 3, P0 = 0.942, Q0 = 0.19),
        nsLFT.ConstantZLoad(; name = "CR4", bus = 4, P0 = 0.478, Q0 = 0.039),
        nsLFT.ConstantZLoad(; name = "CR5", bus = 5, P0 = 0.076, Q0 = 0.016),
        nsLFT.ConstantZLoad(; name = "CR6", bus = 6, P0 = 0.3, Q0 = 0.3),
        nsLFT.ConstantZLoad(; name = "CR9", bus = 9, P0 = 0.295, Q0 = 0.166),
        nsLFT.ConstantZLoad(; name = "CR10", bus = 10, P0 = 0.09, Q0 = 0.058),
        nsLFT.ConstantZLoad(; name = "CR11", bus = 11, P0 = 0.035, Q0 = 0.018),
        nsLFT.ConstantZLoad(; name = "CR12", bus = 12, P0 = 0.061, Q0 = 0.016),
        nsLFT.ConstantZLoad(; name = "CR13", bus = 13, P0 = 0.135, Q0 = 0.058),
        nsLFT.ConstantZLoad(; name = "CR14", bus = 14, P0 = 0.149, Q0 = 0.05),
    ]
end

# The IBR that replaces UG2, with the rated power of UG2 and the parameters in its own base. It injects the power of
# UG2 in the power flow, so the operating point of the case does not change, and it starts at 91% of Imax
function ieee14_ug2_ibr()
    return nsLFT.IBR(;
        name = "IBR2",
        bus = 2,
        Sn = 88.09334324,
        battery = false,
        D = 20.0,
        Kpg = 0.5,
        Kig = 2.0,
        Pmax = 1.0,
        Pmin = 0.0,
        Tp = 0.1,
        Kc = 0.02,
        Kp = 2.0,
        Ki = 10.0,
        Qmax = 1.0,
        Qmin = -1.0,
        Tq = 0.05,
        Vfrz = 0.7,
        Vdip = 0.9,
        Kqv = 2.0,
        Iqh1 = 1.0,
        Iql1 = -1.0,
        Imax = 1.1,
        priority = nsLFT.CurrentPriority.Q,
        Tg = 0.02,
        Rrpwr = 2.0,
        lv0 = 0.4,
        lv1 = 0.8,
        P0 = 0.4,
        Q0 = 0.69417986775,
    )
end

# UG2 leaves the generators and its IBR takes bus 2, which gets the frequency estimation the IBR needs. The
# generators after UG2 move one position up, so the generator index of their buses follows them
function replace_ug2_with_ibr!(buses::Vector{nsLFT.Bus}, generators::Vector{nsLFT.Generator}, ibrs::Vector{nsLFT.IBR})
    deleteat!(generators, 2)
    ibr = ieee14_ug2_ibr()
    push!(ibrs, ibr)
    for bus in buses
        generator_idx = nsLFT.bus_generator_idx(bus)
        if isnothing(generator_idx)
            continue
        end
        if generator_idx == 2
            bus.generator_idx = nothing
        elseif generator_idx > 2
            bus.generator_idx = generator_idx - 1
        end
    end
    ug2_bus = buses[2]
    ug2_bus.ibr_idx = 1
    ug2_bus.freq_estimation = nsLFT.FilteredAngleDerivative(; name = "Bus 02 freq", Tf = 0.05, Tw = 0.1)
    return nothing
end

# the results are written to case_path. With ug2_ibr the generator UG2 is replaced by an IBR
function ieee14_inputs(
    case_path::String;
    perturbations::Vector{<:nsLFT.Perturbation} = nsLFT.Perturbation[],
    generator_model::String = "classic",
    gov::Bool = false,
    avr::Bool = false,
    ug2_ibr::Bool = false,
)
    buses = ieee14_buses()
    generators = ieee14_generators(generator_model, gov, avr)
    ibrs = nsLFT.IBR[]
    if ug2_ibr
        replace_ug2_with_ibr!(buses, generators, ibrs)
    end
    return nsLFT.Inputs(;
        case_path = case_path,
        Sb = 100.0,
        fb = 60.0,
        integration_step = 0.01,
        simulation_horizon = 10.0,
        tolerance = 1e-10,
        buses = buses,
        circuits = ieee14_circuits(),
        generators = generators,
        ibrs = ibrs,
        loads = ieee14_loads(),
        perturbations = perturbations,
    )
end

# perturbations = [nsLFT.ShortCircuit("curto-b14", 14, 1.0, 99999, 0), nsLFT.ShortCircuitRemoval("rm-curto-b14", 1, 1.2), nsLFT.CircuitTrip("trip-linha-b14-13", 17, 1.2)]
# inputs = ieee14_inputs("results/ieee14_classic-curto-b14-trip-linha-b14-13", perturbations = perturbations, generator_model = "classic")
# results = nsLFT.solve(inputs)

# perturbations = [nsLFT.GeneratorTrip(; name = "UG2 trip", generator_idx = 2, time = 1.0)]
# inputs = ieee14_inputs("results/ieee14_classic-trip-ug2", perturbations = perturbations, generator_model = "classic")
# results = nsLFT.solve(inputs)

# perturbations = [nsLFT.ShortCircuit("curto-b14", 14, 1.0, 99999, 0), nsLFT.ShortCircuitRemoval("rm-curto-b14", 1, 1.2), nsLFT.CircuitTrip("trip-linha-b14-13", 17, 1.2)]
# inputs = ieee14_inputs("results/ieee14_1d1q-curto-b14-trip-linha-b14-13", perturbations = perturbations, generator_model = "one_d_one_q")
# results = nsLFT.solve(inputs)

# perturbations = [nsLFT.GeneratorTrip(; name = "UG2 trip", generator_idx = 2, time = 1.0)]
# inputs = ieee14_inputs("results/ieee14_1d1q-trip-ug2", perturbations = perturbations, generator_model = "one_d_one_q")
# results = nsLFT.solve(inputs)

# perturbations = [nsLFT.ShortCircuit("curto-b14", 14, 1.0, 99999, 0), nsLFT.ShortCircuitRemoval("rm-curto-b14", 1, 1.2), nsLFT.CircuitTrip("trip-linha-b14-13", 17, 1.2)]
# inputs = ieee14_inputs("results/ieee14_1d1q-gov-curto-b14-trip-linha-b14-13", perturbations = perturbations, generator_model = "one_d_one_q", gov = true)
# results = nsLFT.solve(inputs)

# perturbations = [nsLFT.GeneratorTrip(; name = "UG2 trip", generator_idx = 2, time = 1.0)]
# inputs = ieee14_inputs("results/ieee14_1d1q-gov-trip-ug2", perturbations = perturbations, generator_model = "one_d_one_q", gov = true)
# results = nsLFT.solve(inputs)

nothing
