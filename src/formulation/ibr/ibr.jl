# the converter takes Pref and Qref at ts + 1 from the controls, so it comes after them
function predict_ibr!(
    inputs::Inputs,
    ibr::IBR,
    state::IBRState,
    freq_estimation_state::BusFreqEstimationState,
    ts::Int,
)
    predict_active_power_control!(inputs, ibr, state, freq_estimation_state, ts)
    predict_reactive_power_control!(inputs, ibr, state, ts)
    predict_converter!(inputs, ibr, state, ts)
    return nothing
end

function step_ibr!(
    inputs::Inputs,
    ibr::IBR,
    state::IBRState,
    freq_estimation_state::BusFreqEstimationState,
    ts::Int,
)
    step_active_power_control!(inputs, ibr, state, freq_estimation_state, ts)
    step_reactive_power_control!(inputs, ibr, state, ts)
    step_converter!(inputs, ibr, state, ts)
    return nothing
end
