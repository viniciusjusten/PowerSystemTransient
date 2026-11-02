function initialize_mechanical_power!(::Generator, state::GeneratorState)
    state.Pmec .= state.Pele[1]
    return nothing
end

# Pmec_1 starts at the mechanical power, and Pref is chosen so that it starts in steady state, dPmec_1/dt = 0 at
# t = 0, which gives Pref = Pmec_1 - (omega_ref - omega) / R
function initialize_governor!(
    inputs::Inputs,
    generator::Generator,
    governor::Droop1stOrder,
    governor_state::Droop1stOrderState,
    state::GeneratorState,
)
    R = governor_R(inputs, generator, governor)
    omega_ref = governor_omega_ref(inputs, governor)
    governor_state.Pmec_1 .= state.Pmec[1]
    governor_state.Pref .= governor_state.Pmec_1[1] - (omega_ref - state.omega[1]) / R
    return nothing
end

# without a governor the mechanical power stays at its initial value and there is no state to initialize
function initialize_governor!(::Inputs, ::Generator, ::Nothing, ::Nothing, ::GeneratorState)
    return nothing
end

function initialize_mechanical_powers!(inputs::Inputs, results::Results)
    for (generator, state) in zip(inputs.generators, results.generator)
        initialize_mechanical_power!(generator, state)
        initialize_governor!(inputs, generator, generator_governor(generator), state.governor, state)
    end
    return nothing
end
