mutable struct IBRState
    # Terminal
    Vt::Vector{Float64} # terminal voltage magnitude, in pu

    # Active power control
    Pset::Vector{Float64} # active power setpoint, the power the IBR injects in the network of nsLFT at t = 0, in pu
    Pmed::Vector{Float64} # active power injected at the terminal, Vt * Ip, in pu
    xp::Vector{Float64} # proportional part of the active power PI, in pu
    yp::Vector{Float64} # integral part of the active power PI, the state of its integrator, in pu
    Pext::Vector{Float64} # output of the active power PI after the limits, in pu
    Pref::Vector{Float64} # active power reference, the output of the lag 1 / (1 + s Tp), in pu

    # Reactive power control
    Vref::Vector{Float64} # voltage setpoint, chosen so that the voltage error starts at zero, in pu
    Qmed::Vector{Float64} # reactive power injected at the terminal, Vt * Iq, in pu
    xq::Vector{Float64} # proportional part of the voltage PI, in pu
    yq::Vector{Float64} # integral part of the voltage PI, the state of its integrator, in pu
    Qext::Vector{Float64} # output of the voltage PI after the limits, in pu
    Qref::Vector{Float64} # reactive power reference, the output of the lag 1 / (1 + s Tq), in pu

    # Current logic
    iq_inj::Vector{Float64} # reactive current injection during the voltage dip, in pu
    Ip_cmd_pre::Vector{Float64} # active current command before the current limit, Pref / Vt, in pu
    Iq_cmd_pre::Vector{Float64} # reactive current command before the current limit, iq_inj + Qref / Vt, in pu
    Ip_max::Vector{Float64} # upper limit of the active current command, in pu
    Iq_max::Vector{Float64} # limit of the magnitude of the reactive current command, in pu
    Ip_cmd::Vector{Float64} # active current command after the current limit, in pu
    Iq_cmd::Vector{Float64} # reactive current command after the current limit, in pu

    # Converter
    Ip::Vector{Float64} # active current, the output of the lag 1 / (1 + s Tg) with the rise limited by Rrpwr, in pu
    Iq::Vector{Float64} # reactive current, the output of the lag 1 / (1 + s Tg), in pu
    Ip_ef::Vector{Float64} # active current injected in the network, Ip times the low voltage gain, in pu
end

function IBRState(num_steps::Int)
    # NaN keeps a step not yet computed from passing a convergence check by accident
    return IBRState(
        fill(NaN, num_steps),
        fill(NaN, num_steps),
        fill(NaN, num_steps),
        fill(NaN, num_steps),
        fill(NaN, num_steps),
        fill(NaN, num_steps),
        fill(NaN, num_steps),
        fill(NaN, num_steps),
        fill(NaN, num_steps),
        fill(NaN, num_steps),
        fill(NaN, num_steps),
        fill(NaN, num_steps),
        fill(NaN, num_steps),
        fill(NaN, num_steps),
        fill(NaN, num_steps),
        fill(NaN, num_steps),
        fill(NaN, num_steps),
        fill(NaN, num_steps),
        fill(NaN, num_steps),
        fill(NaN, num_steps),
        fill(NaN, num_steps),
        fill(NaN, num_steps),
        fill(NaN, num_steps),
    )
end

convergence_variables(::IBRState) = [:yp, :Pref, :yq, :Qref, :Ip, :Iq]
