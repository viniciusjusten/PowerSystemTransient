# Static 1st order voltage regulator model
function initialize_voltage_regulator!(
    inputs::Inputs,
    generator::Generator,
    voltage_regulator::Static1stOrder,
    voltage_regulator_state::Static1stOrderState,
    state::GeneratorState,
)
    efd = state.efd[1]
    if !(efd >= voltage_regulator_Lmn(voltage_regulator) && efd <= voltage_regulator_Lmx(voltage_regulator))
        error("Initial field voltage (efd = $efd) is outside the limits of the voltage regulator (Lmn = $(voltage_regulator_Lmn(voltage_regulator)), Lmx = $(voltage_regulator_Lmx(voltage_regulator)))")
    end
    voltage_regulator_state.efdu[1] = efd
    voltage_regulator_state.Vref[1] = state.Vt[1] + efd / voltage_regulator_K(voltage_regulator)
    for ts in 2:length(voltage_regulator_state.Vref)
        voltage_regulator_state.Vref[ts] = voltage_regulator_state.Vref[1]
    end
    return nothing
end

# without a voltage regulator the field voltage stays at its initial value and there is no state to initialize
function initialize_voltage_regulator!(::Inputs, ::Generator, ::Nothing, ::Nothing, ::GeneratorState)
    return nothing
end

function initialize_voltage_regulators!(inputs::Inputs, results::Results)
    for (generator, state) in zip(inputs.generators, results.generator)
        voltage_regulator = generator_voltage_regulator(generator)
        initialize_voltage_regulator!(inputs, generator, voltage_regulator, state.voltage_regulator, state)
    end
    return nothing
end
