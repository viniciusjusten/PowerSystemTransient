# The perturbations of the IEEE 14 test cases, shared by the tests and by write_references.jl

# trip of the generator UG2 at t = 1 s
function ieee14_ug2_trip_perturbations()
    return PST.Perturbation[PST.GeneratorTrip(; name = "UG2 trip", generator_idx = 2, time = 1.0)]
end

# short circuit at bus 14 at t = 1 s, cleared at t = 1.2 s by opening the line LT13-14, which is circuit 17.
# The short circuit is a large conductance, so the voltage of bus 14 is close to zero while it lasts
function ieee14_short_circuit_perturbations()
    return PST.Perturbation[
        PST.ShortCircuit(;
            name = "curto-b14",
            bus_idx = 14,
            time = 1.0,
            fault_conductance = 99999.0,
            fault_susceptance = 0.0,
        ),
        PST.ShortCircuitRemoval(; name = "rm-curto-b14", short_circuit_idx = 1, time = 1.2),
        PST.CircuitTrip(; name = "trip-linha-b14-13", circuit_idx = 17, time = 1.2),
    ]
end
