module PowerSystemTransient

using EnumX
using LinearAlgebra
using SparseArrays

include("utils.jl")

include("collections/bus_freq_estimation.jl")
include("collections/bus.jl")
include("collections/circuit.jl")
include("collections/generator/governor.jl")
include("collections/generator/voltage_regulator.jl")
include("collections/generator/generator.jl")
include("collections/ibr.jl")
include("collections/load.jl")
include("collections/perturbation.jl")
include("collections/companion.jl")
include("collections/network.jl")

include("states/control.jl")
include("states/bus_freq_estimation.jl")
include("states/bus.jl")
include("states/generator/governor.jl")
include("states/generator/voltage_regulator.jl")
include("states/generator/generator.jl")
include("states/ibr.jl")

include("inputs.jl")
include("results/results.jl")
include("results/write.jl")
include("results/case_description.jl")

include("formulation/network/phasor.jl")
include("formulation/network/rms.jl")
include("formulation/network/companion.jl")
include("formulation/network/dynamic_phasor.jl")
include("formulation/generator/generator_swing.jl")
include("formulation/generator/classic_generator.jl")
include("formulation/generator/one_d_one_q_generator.jl")
include("formulation/generator/governor.jl")
include("formulation/generator/voltage_regulator.jl")
include("formulation/generator/generator.jl")
include("formulation/bus_freq_estimation.jl")
include("formulation/ibr/active_power.jl")
include("formulation/ibr/reactive_power.jl")
include("formulation/ibr/current_logic.jl")
include("formulation/ibr/converter.jl")
include("formulation/ibr/ibr.jl")

include("initialization/generator/generator.jl")
include("initialization/generator/governor.jl")
include("initialization/generator/voltage_regulator.jl")
include("initialization/bus_freq_estimation.jl")
include("initialization/ibr/active_power.jl")
include("initialization/ibr/reactive_power.jl")
include("initialization/ibr/current_logic.jl")
include("initialization/ibr/converter.jl")
include("initialization/initialization.jl")

include("solver.jl")

greet() = print("Hello World!")

end # module PowerSystemTransient
