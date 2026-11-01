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
