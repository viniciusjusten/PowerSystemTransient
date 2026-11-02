# Static 1st order voltage regulator model
function deriv_efdu(
    voltage_regulator::Static1stOrder,
    voltage_regulator_state::Static1stOrderState,
    state::GeneratorState,
    ts::Int,
)
    return 1 / voltage_regulator_T(voltage_regulator) * (
        voltage_regulator_K(voltage_regulator) *
        (voltage_regulator_state.Vref[ts] - state.Vt[ts]) -
        voltage_regulator_state.efdu[ts]
    )
end

function clamp_efdu(voltage_regulator::Static1stOrder, voltage_regulator_state::Static1stOrderState, ts::Int)
    return clamp(
        voltage_regulator_state.efdu[ts],
        voltage_regulator_Lmn(voltage_regulator),
        voltage_regulator_Lmx(voltage_regulator)
    )
end

function predict_efd!(
    inputs::Inputs,
    generator::Generator,
    voltage_regulator::Static1stOrder,
    voltage_regulator_state::Static1stOrderState,
    state::GeneratorState,
    ts::Int,
)
    # predict the next efdu value using the Euler method
    voltage_regulator_state.efdu[ts+1] =
        voltage_regulator_state.efdu[ts] +
        integration_step(inputs) * deriv_efdu(voltage_regulator, voltage_regulator_state, state, ts)
    state.efd[ts+1] = clamp_efdu(voltage_regulator, voltage_regulator_state, ts+1)
    return nothing
end

function step_efd!(
    inputs::Inputs,
    generator::Generator,
    voltage_regulator::Static1stOrder,
    voltage_regulator_state::Static1stOrderState,
    state::GeneratorState,
    ts::Int,
)
    # compute the next efdu value using the trapezoidal rule
    voltage_regulator_state.efdu[ts+1] =
        voltage_regulator_state.efdu[ts] +
        integration_step(inputs) / 2 * (
            deriv_efdu(voltage_regulator, voltage_regulator_state, state, ts) +
            deriv_efdu(voltage_regulator, voltage_regulator_state, state, ts+1)
        )
    state.efd[ts+1] = clamp_efdu(voltage_regulator, voltage_regulator_state, ts+1)
    return nothing
end

# without a voltage regulator the field voltage stays constant
function predict_efd!(::Inputs, ::ClassicGenerator, ::Nothing, ::Nothing, ::GeneratorState, ::Int)
    return nothing
end

function step_efd!(::Inputs, ::ClassicGenerator, ::Nothing, ::Nothing, ::GeneratorState, ::Int)
    return nothing
end

function predict_efd!(::Inputs, ::OneDOneQGenerator, ::Nothing, ::Nothing, state::GeneratorState, ts::Int)
    state.efd[ts+1] = state.efd[ts]
    return nothing
end

function step_efd!(::Inputs, ::OneDOneQGenerator, ::Nothing, ::Nothing, state::GeneratorState, ts::Int)
    state.efd[ts+1] = state.efd[ts]
    return nothing
end
