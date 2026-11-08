using Test
import PowerSystemTransient as PST

include(joinpath(@__DIR__, "..", "scripts", "read_csv.jl"))
include(joinpath(@__DIR__, "..", "scripts", "ieee14.jl"))
include(joinpath(@__DIR__, "trajectories.jl"))
include(joinpath(@__DIR__, "ieee14_perturbations.jl"))

@testset "PowerSystemTransient" begin
    include(joinpath("ieee14_no_perturbation", "ieee14.jl"))
    include(joinpath("ieee14_ug2_trip", "ieee14.jl"))
    include(joinpath("ieee14_short_circuit", "ieee14.jl"))
    include(joinpath("ieee14_one_d_one_q_ug2_trip", "ieee14.jl"))
    include(joinpath("ieee14_one_d_one_q_short_circuit", "ieee14.jl"))
    include(joinpath("ieee14_one_d_one_q_governor_ug2_trip", "ieee14.jl"))
    include(joinpath("ieee14_one_d_one_q_governor_short_circuit", "ieee14.jl"))
    include(joinpath("ieee14_one_d_one_q_governor_avr_ug2_trip", "ieee14.jl"))
    include(joinpath("ieee14_one_d_one_q_governor_avr_short_circuit", "ieee14.jl"))
    include(joinpath("ieee14_one_d_one_q_governor_avr_freq_estimation_ug2_trip", "ieee14.jl"))
    include(joinpath("ieee14_one_d_one_q_governor_avr_ibr_short_circuit", "ieee14.jl"))
    include(joinpath("ieee14_one_d_one_q_governor_avr_ibr_trip", "ieee14.jl"))

    # the dynamic phasor network with one d one q generators, governors and voltage regulators, with and without UG2
    # replaced by an IBR, under the short circuit and the trip of UG2 or of the IBR that replaces it
    include(joinpath("ieee14_dp_one_d_one_q_governor_avr_ug2_trip", "ieee14.jl"))
    include(joinpath("ieee14_dp_one_d_one_q_governor_avr_short_circuit", "ieee14.jl"))
    include(joinpath("ieee14_dp_one_d_one_q_governor_avr_ibr_short_circuit", "ieee14.jl"))
    include(joinpath("ieee14_dp_one_d_one_q_governor_avr_ibr_trip", "ieee14.jl"))
end
