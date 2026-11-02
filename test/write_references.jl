# Writes the reference trajectories of the IEEE 14 cases with perturbations from the current model.
# The tests never run this script. It is run by hand, only after a change that is meant to alter the results:
#   julia --project test/write_references.jl
# With case names as arguments only those cases are written, so a new case gets its reference without rewriting
# the others:
#   julia --project test/write_references.jl ieee14_one_d_one_q_governor_avr_ibr_short_circuit

using Test
using nsLFT

include(joinpath(@__DIR__, "..", "scripts", "read_csv.jl"))
include(joinpath(@__DIR__, "..", "scripts", "ieee14.jl"))
include(joinpath(@__DIR__, "trajectories.jl"))
include(joinpath(@__DIR__, "ieee14_perturbations.jl"))

# each case is the test folder, its perturbations, its generator model, whether it has governors and voltage regulators
# and whether UG2 is replaced by an IBR
cases = [
    ("ieee14_ug2_trip", ieee14_ug2_trip_perturbations(), "classic", false, false, false),
    ("ieee14_short_circuit", ieee14_short_circuit_perturbations(), "classic", false, false, false),
    ("ieee14_one_d_one_q_ug2_trip", ieee14_ug2_trip_perturbations(), "one_d_one_q", false, false, false),
    ("ieee14_one_d_one_q_short_circuit", ieee14_short_circuit_perturbations(), "one_d_one_q", false, false, false),
    ("ieee14_one_d_one_q_governor_ug2_trip", ieee14_ug2_trip_perturbations(), "one_d_one_q", true, false, false),
    ("ieee14_one_d_one_q_governor_short_circuit", ieee14_short_circuit_perturbations(), "one_d_one_q", true, false, false),
    ("ieee14_one_d_one_q_governor_avr_ug2_trip", ieee14_ug2_trip_perturbations(), "one_d_one_q", true, true, false),
    ("ieee14_one_d_one_q_governor_avr_short_circuit", ieee14_short_circuit_perturbations(), "one_d_one_q", true, true, false),
    ("ieee14_one_d_one_q_governor_avr_ibr_short_circuit", ieee14_short_circuit_perturbations(), "one_d_one_q", true, true, true),
]

for (case_name, perturbations, generator_model, gov, avr, ug2_ibr) in cases
    if !isempty(ARGS) && !(case_name in ARGS)
        continue
    end
    case_path = joinpath(@__DIR__, case_name)
    inputs = ieee14_inputs(
        case_path;
        perturbations = perturbations,
        generator_model = generator_model,
        gov = gov,
        avr = avr,
        ug2_ibr = ug2_ibr,
    )
    results = nsLFT.solve(inputs)

    trajectory_names, trajectories = model_trajectories(inputs, results)
    time = nsLFT.time_values(inputs)
    reference_path = joinpath(case_path, "reference")
    mkpath(reference_path)
    nsLFT.write_csv(joinpath(reference_path, "trajectories.csv"), time, trajectory_names, trajectories)
    println("Reference of $case_name written to $reference_path")
end
