mutable struct IBRState
    # Active power control
    Pset::Vector{Float64} # active power setpoint, the power the IBR injects in the network of nsLFT at t = 0, in pu
    Pmed::Vector{Float64} # active power injected at the terminal, Vt * Ip, in pu
    xp::Vector{Float64} # proportional part of the active power PI, in pu
    yp::Vector{Float64} # integral part of the active power PI, the state of its integrator, in pu
    Pext::Vector{Float64} # output of the active power PI after the limits, in pu
    Pref::Vector{Float64} # active power reference, the output of the lag 1 / (1 + s Tp), in pu
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
    )
end
