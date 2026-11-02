function time_values(inputs::Inputs)
    return [time_at(inputs, ts) for ts in 1:num_time_steps(inputs)+1]
end

# the variables of a state as pairs of name and value, including the variables of the control states nested in it,
# so the states of the governor and of the voltage regulator are written with the generator they belong to
function state_variables(state)
    variables = Pair{Symbol, Any}[]
    for field in fieldnames(typeof(state))
        value = getfield(state, field)
        if value isa ControlState
            nested_variables = state_variables(value)
            append!(variables, nested_variables)
        else
            push!(variables, field => value)
        end
    end
    return variables
end

# the time series of a state as pairs of name and values, including those of its control states
function time_series(state)
    series = Pair{Symbol, Vector{Float64}}[]
    for (name, value) in state_variables(state)
        if value isa Vector{Float64}
            push!(series, name => value)
        end
    end
    return series
end

# collects the names of the time series of all states, so element types with different models
# can share a file for the quantities they have in common
function time_series_fields(states::Vector)
    fields = Symbol[]
    for state in states
        for (name, _) in time_series(state)
            if !(name in fields)
                push!(fields, name)
            end
        end
    end
    return fields
end

# the units each time series field is written in, as (unit name, factor that converts the stored value to it).
# The stored values are in pu, except the angles delta and theta_estim, which have no pu value and are stored in rad.
# A field with no units is not written
function output_units(inputs::Inputs, field::Symbol)
    if field == :Vr || field == :Vi
        # the buses have no base voltage yet, so the voltages are only written in pu
        return [("pu", 1.0)]
    elseif field == :delta || field == :theta_estim
        return [("rad", 1.0), ("deg", 180 / pi)]
    elseif field == :x_theta
        # x_theta is an internal state of the filtered derivative of the frequency estimation, so it is not written
        return Tuple{String, Float64}[]
    elseif field == :omega || field == :d_omega
        return [("pu", 1.0), ("Hz", fb(inputs))]
    elseif field == :Pmec_1 || field == :Pmec || field == :Pref || field == :Pele
        return [("pu", 1.0), ("MW", Sb(inputs))]
    elseif field == :Pset || field == :Pmed || field == :xp || field == :yp || field == :Pext
        return [("pu", 1.0), ("MW", Sb(inputs))]
    elseif field == :Qmed || field == :xq || field == :yq || field == :Qext || field == :Qref
        return [("pu", 1.0), ("Mvar", Sb(inputs))]
    elseif field == :Eq_prime || field == :ed_prime || field == :eq_prime || field == :efd || field == :efdu || field == :Vref || field == :Vt
        return [("pu", 1.0)]
    elseif field == :id || field == :iq
        return [("pu", 1.0)]
    elseif field == :iq_inj || field == :Ip_cmd_pre || field == :Iq_cmd_pre || field == :Ip_max || field == :Iq_max || field == :Ip_cmd || field == :Iq_cmd
        return [("pu", 1.0)]
    elseif field == :Ip || field == :Iq || field == :Ip_ef
        return [("pu", 1.0)]
    end
    error("No output unit is defined for the field $field.")
end

function write_csv(file_path::String, time::Vector{Float64}, column_names::Vector{String}, columns::Vector{Vector{Float64}})
    io = open(file_path, "w")
    try
        println(io, join(["time_s"; column_names], ","))
        for step in eachindex(time)
            row = [time[step]; [column[step] for column in columns]]
            println(io, join(row, ","))
        end
    finally
        close(io)
    end
    return nothing
end

function write_class_results(
    inputs::Inputs,
    output_path::String,
    class_name::String,
    time::Vector{Float64},
    element_names::Vector{String},
    states::Vector,
)
    for field in time_series_fields(states)
        for (unit, factor) in output_units(inputs, field)
            column_names = String[]
            columns = Vector{Float64}[]
            for (element_idx, state) in enumerate(states)
                for (name, values) in time_series(state)
                    if name == field
                        push!(column_names, element_names[element_idx])
                        push!(columns, values .* factor)
                    end
                end
            end
            write_csv(joinpath(output_path, "$(class_name)_$(field)_$(unit).csv"), time, column_names, columns)
        end
    end
    return nothing
end

# the bus states keep the voltages in rectangular form, so the magnitude and the angle are computed here.
# The angle is in the interval from -pi to pi, as returned by angle
function write_bus_polar_voltages(
    output_path::String,
    time::Vector{Float64},
    bus_names::Vector{String},
    states::Vector{BusState},
)
    magnitudes = Vector{Float64}[]
    angles_rad = Vector{Float64}[]
    angles_deg = Vector{Float64}[]
    for state in states
        V = state.Vr .+ im .* state.Vi
        theta = angle.(V)
        push!(magnitudes, abs.(V))
        push!(angles_rad, theta)
        push!(angles_deg, rad2deg.(theta))
    end
    write_csv(joinpath(output_path, "bus_V_abs_pu.csv"), time, bus_names, magnitudes)
    write_csv(joinpath(output_path, "bus_theta_rad.csv"), time, bus_names, angles_rad)
    write_csv(joinpath(output_path, "bus_theta_deg.csv"), time, bus_names, angles_deg)
    return nothing
end

function write_results(inputs::Inputs, results::Results, output_path::String)
    mkpath(output_path)
    time = time_values(inputs)

    bus_names = [bus_name(bus) for bus in inputs.buses]
    write_class_results(inputs, output_path, "bus", time, bus_names, results.bus)
    write_bus_polar_voltages(output_path, time, bus_names, results.bus)

    generator_names = [generator_name(generator) for generator in inputs.generators]
    write_class_results(inputs, output_path, "generator", time, generator_names, results.generator)

    ibr_names = [ibr_name(ibr) for ibr in inputs.ibrs]
    write_class_results(inputs, output_path, "ibr", time, ibr_names, results.ibr)

    write_case_description(inputs, results, output_path)

    return nothing
end
