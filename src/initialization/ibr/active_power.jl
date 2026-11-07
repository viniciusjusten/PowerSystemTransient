# the active power control of the IBR takes the frequency from the estimation of its bus, so every bus with an IBR
# must estimate its frequency
function validate_ibr_freq_estimations(inputs::Inputs)
    for ibr in inputs.ibrs
        bus_idx = ibr_bus(ibr)
        bus = get_bus(inputs, bus_idx)
        if !has_freq_estimation(bus)
            error("The IBR $(ibr_name(ibr)) is connected to the bus $(bus_name(bus)), which has no frequency estimation.")
        end
    end
    return nothing
end

# The setpoint is the power the IBR injects in the network of PowerSystemTransient at t = 0, rather than P0 of the
# power flow, which the network only reproduces to the precision of the case data. So Pmed at t = 0 must come from the
# network solution, which needs the converter initialized before it. With f = 1 and Pset = Pmed the error is zero, so
# xp = 0 and the integrator holds the whole output, yp = Pset, which gives Pext = Pref = Pset
function initialize_active_power_control!(inputs::Inputs, ibr::IBR, state::IBRState)
    Pset = state.Pmed[1]
    Pmin = ibr_Pmin(inputs, ibr)
    Pmax = ibr_Pmax(inputs, ibr)
    if !(Pset >= Pmin && Pset <= Pmax)
        error("Initial active power of the IBR $(ibr_name(ibr)) (Pmed = $Pset) is outside the limits of the active power control (Pmin = $Pmin, Pmax = $Pmax)")
    end
    state.Pset .= Pset
    state.xp[1] = 0.0
    state.yp[1] = Pset
    state.Pext[1] = Pset
    state.Pref[1] = Pset
    return nothing
end
