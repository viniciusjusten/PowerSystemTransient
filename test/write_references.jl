# Writes the reference trajectories of the IEEE 14 cases with perturbations from the current model.
# The tests never run this script. It is run by hand, only after a change that is meant to alter the results:
#   julia --project test/write_references.jl
# With case names as arguments only those cases are written, so a new case gets its reference without rewriting
# the others:
#   julia --project test/write_references.jl ieee14_one_d_one_q_governor_avr_ibr_short_circuit

using Test
import PowerSystemTransient as PST

include(joinpath(@__DIR__, "..", "scripts", "read_csv.jl"))
include(joinpath(@__DIR__, "..", "scripts", "ieee14.jl"))
include(joinpath(@__DIR__, "trajectories.jl"))
include(joinpath(@__DIR__, "ieee14_perturbations.jl"))

# each case is the test folder, its perturbations, its generator model, whether it has governors and voltage
# regulators, whether UG2 is replaced by an IBR and whether the network is the dynamic phasor one
cases = [
    ("ieee14_ug2_trip", ieee14_ug2_trip_perturbations(), "classic", false, false, false, false),
    ("ieee14_short_circuit", ieee14_short_circuit_perturbations(), "classic", false, false, false, false),
    ("ieee14_one_d_one_q_ug2_trip", ieee14_ug2_trip_perturbations(), "one_d_one_q", false, false, false, false),
    ("ieee14_one_d_one_q_short_circuit", ieee14_short_circuit_perturbations(), "one_d_one_q", false, false, false, false),
    ("ieee14_one_d_one_q_governor_ug2_trip", ieee14_ug2_trip_perturbations(), "one_d_one_q", true, false, false, false),
    ("ieee14_one_d_one_q_governor_short_circuit", ieee14_short_circuit_perturbations(), "one_d_one_q", true, false, false, false),
    ("ieee14_one_d_one_q_governor_avr_ug2_trip", ieee14_ug2_trip_perturbations(), "one_d_one_q", true, true, false, false),
    ("ieee14_one_d_one_q_governor_avr_short_circuit", ieee14_short_circuit_perturbations(), "one_d_one_q", true, true, false, false),
    ("ieee14_one_d_one_q_governor_avr_ibr_short_circuit", ieee14_short_circuit_perturbations(), "one_d_one_q", true, true, true, false),
    ("ieee14_dp_one_d_one_q_governor_avr_ug2_trip", ieee14_ug2_trip_perturbations(), "one_d_one_q", true, true, false, true),
    ("ieee14_dp_one_d_one_q_governor_avr_short_circuit", ieee14_short_circuit_perturbations(), "one_d_one_q", true, true, false, true),
    ("ieee14_dp_one_d_one_q_governor_avr_ibr_short_circuit", ieee14_short_circuit_perturbations(), "one_d_one_q", true, true, true, true),
]

for (case_name, perturbations, generator_model, gov, avr, ug2_ibr, dynamic_phasor) in cases
    if !isempty(ARGS) && !(case_name in ARGS)
        continue
    end

    # The RMS network takes a 10 ms step over 10 s. The dynamic phasor network takes a 1 ms step to follow the
    # transients of the stators and of the network, over 3 s, which covers the perturbations at 1 s and 1.2 s and keeps
    # the size of the reference close to the RMS ones
    network = PST.NetworkRMS()
    integration_step = 0.01
    simulation_horizon = 10.0
    if dynamic_phasor
        network = PST.NetworkDP()
        integration_step = 0.001
        simulation_horizon = 3.0
    end

    case_path = joinpath(@__DIR__, case_name)
    inputs = ieee14_inputs(
        case_path;
        perturbations = perturbations,
        generator_model = generator_model,
        gov = gov,
        avr = avr,
        ug2_ibr = ug2_ibr,
        network = network,
        integration_step = integration_step,
        simulation_horizon = simulation_horizon,
    )
    results = PST.solve(inputs)

    trajectory_names, trajectories = model_trajectories(inputs, results)
    time = PST.time_values(inputs)
    reference_path = joinpath(case_path, "reference")
    mkpath(reference_path)
    PST.write_csv(joinpath(reference_path, "trajectories.csv"), time, trajectory_names, trajectories)
    println("Reference of $case_name written to $reference_path")
end
