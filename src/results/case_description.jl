# A minimal JSON writer for the case description. Objects are vectors of pairs, so the keys keep their order
function write_json(io::IO, object::Vector{<:Pair})
    print(io, "{")
    for (pair_idx, pair) in enumerate(object)
        if pair_idx > 1
            print(io, ",")
        end
        key = string(first(pair))
        write_json(io, key)
        print(io, ":")
        write_json(io, last(pair))
    end
    print(io, "}")
    return nothing
end

function write_json(io::IO, values::AbstractVector)
    print(io, "[")
    for (value_idx, value) in enumerate(values)
        if value_idx > 1
            print(io, ",")
        end
        write_json(io, value)
    end
    print(io, "]")
    return nothing
end

function write_json(io::IO, text::AbstractString)
    print(io, "\"")
    for character in text
        if character == '"'
            print(io, "\\\"")
        elseif character == '\\'
            print(io, "\\\\")
        elseif character == '<'
            # the description is embedded in the dashboard page, where "</script>" would end its block
            print(io, "\\u003c")
        elseif character < ' '
            code = string(UInt16(character); base = 16, pad = 4)
            print(io, "\\u", code)
        else
            print(io, character)
        end
    end
    print(io, "\"")
    return nothing
end

function write_json(io::IO, value::Bool)
    print(io, value)
    return nothing
end

function write_json(io::IO, value::Real)
    # JSON has no NaN or Inf, such as the rated voltage the case does not give
    if !isfinite(value)
        print(io, "null")
        return nothing
    end
    print(io, value)
    return nothing
end

function write_json(io::IO, ::Nothing)
    print(io, "null")
    return nothing
end

# a quantity is a key with its value in one or more units, such as an angle in rad and in deg
function quantity(key::String, values::Vector{Tuple{Float64, String}})
    entries = Any[]
    for (value, unit) in values
        push!(entries, Pair{String, Any}["value" => value, "unit" => unit])
    end
    return Pair{String, Any}["key" => key, "values" => entries]
end

function quantity(key::String, value::Real, unit::String)
    return quantity(key, [(Float64(value), unit)])
end

function angle_quantity(key::String, angle_rad::Float64)
    angle_deg = rad2deg(angle_rad)
    return quantity(key, [(angle_rad, "rad"), (angle_deg, "deg")])
end

function power_quantity(inputs::Inputs, key::String, value_pu::Float64, unit::String)
    value = value_pu * Sb(inputs)
    return quantity(key, [(value, unit), (value_pu, "pu")])
end

function voltage_at_start(state::BusState)
    return state.Vr[1] + im * state.Vi[1]
end

function system_description(inputs::Inputs)
    return Any[
        quantity("Sb", Sb(inputs), "MVA"),
        quantity("fb", fb(inputs), "Hz"),
        quantity("integration_step", integration_step(inputs), "s"),
        quantity("simulation_horizon", simulation_horizon(inputs), "s"),
        quantity("tolerance", tolerance(inputs), ""),
        quantity("max_iterations", max_iterations(inputs), ""),
    ]
end

function bus_description(inputs::Inputs, bus::Bus, state::BusState)
    V = voltage_at_start(state)
    magnitude = abs(V)
    theta = angle(V)
    B = bus_shunt_susceptance(bus)

    initial = Any[
        quantity("V", magnitude, "pu"),
        angle_quantity("theta", theta),
    ]
    parameters = Any[
        quantity("V_pf", bus_voltage(bus), "pu"),
        angle_quantity("theta_pf", bus_angle(bus)),
        power_quantity(inputs, "B_shunt", B, "Mvar"),
    ]
    return Pair{String, Any}[
        "name" => bus_name(bus),
        "initial" => initial,
        "parameters" => parameters,
    ]
end

# the flows use the same pi model and tap convention as admittance_matrix, with every circuit in service at t = 0
function circuit_description(inputs::Inputs, circuit::Circuit, from_state::BusState, to_state::BusState)
    V_from = voltage_at_start(from_state)
    V_to = voltage_at_start(to_state)
    tap = circuit_tap_ratio(circuit)
    y = admittance(circuit_resistance(circuit), circuit_reactance(circuit))
    y_shunt = circuit_shunt_conductance(circuit) + im * circuit_shunt_susceptance(circuit)

    I_from = (y * tap^2 + y_shunt / 2) * V_from - y * tap * V_to
    I_to = (y + y_shunt / 2) * V_to - y * tap * V_from
    S_from = V_from * conj(I_from)
    S_to = V_to * conj(I_to)
    S_loss = S_from + S_to

    # the circuit does not record whether it is a line or a transformer, so a tap other than 1 marks a transformer
    circuit_type = "line"
    if tap != 1.0
        circuit_type = "transformer"
    end

    initial = Any[
        power_quantity(inputs, "P_from", real(S_from), "MW"),
        power_quantity(inputs, "Q_from", imag(S_from), "Mvar"),
        power_quantity(inputs, "P_to", real(S_to), "MW"),
        power_quantity(inputs, "Q_to", imag(S_to), "Mvar"),
        power_quantity(inputs, "P_loss", real(S_loss), "MW"),
        power_quantity(inputs, "Q_loss", imag(S_loss), "Mvar"),
        quantity("I_from", abs(I_from), "pu"),
    ]
    parameters = Any[
        quantity("R", circuit_resistance(circuit), "pu"),
        quantity("X", circuit_reactance(circuit), "pu"),
        quantity("G", circuit_shunt_conductance(circuit), "pu"),
        quantity("B", circuit_shunt_susceptance(circuit), "pu"),
        quantity("tap_ratio", tap, "pu"),
    ]
    return Pair{String, Any}[
        "name" => circuit_name(circuit),
        "from" => circuit_from_bus_idx(circuit),
        "to" => circuit_to_bus_idx(circuit),
        "type" => circuit_type,
        "initial" => initial,
        "parameters" => parameters,
    ]
end

# the parameters are written in the machine base, as the case data gives them
function generator_parameters(generator::ClassicGenerator)
    return Any[
        quantity("Sn", generator_Sn(generator), "MVA"),
        quantity("Vn", generator_Vn(generator), "kV"),
        quantity("Xd_prime", generator.Xd_prime, "pu"),
        quantity("Ra", generator.Ra, "pu"),
        quantity("H", generator.H, "s"),
        quantity("D", generator.D, "pu"),
    ]
end

function generator_parameters(generator::OneDOneQGenerator)
    return Any[
        quantity("Sn", generator_Sn(generator), "MVA"),
        quantity("Vn", generator_Vn(generator), "kV"),
        quantity("Xd", generator.Xd, "pu"),
        quantity("Xd_prime", generator.Xd_prime, "pu"),
        quantity("Xq", generator.Xq, "pu"),
        quantity("Xq_prime", generator.Xq_prime, "pu"),
        quantity("Ra", generator.Ra, "pu"),
        quantity("Td0_prime", generator_Td0_prime(generator), "s"),
        quantity("Tq0_prime", generator_Tq0_prime(generator), "s"),
        quantity("H", generator.H, "s"),
        quantity("D", generator.D, "pu"),
    ]
end

# every state variable of the model and of its controls is listed with its value at t = 0, so a new model needs no
# change here
function state_initial_values(inputs::Inputs, state)
    values = Any[]
    for (field, value) in state_variables(state)
        initial = NaN
        if value isa Vector{Float64}
            initial = value[1]
        elseif value isa Float64
            initial = value
        else
            continue
        end
        units = Tuple{Float64, String}[]
        for (unit, factor) in output_units(inputs, field)
            push!(units, (initial * factor, unit))
        end
        push!(values, quantity(string(field), units))
    end
    return values
end

function generator_description(inputs::Inputs, generator::Generator, state::GeneratorState, bus_state::BusState)
    # the terminal power comes from the dq currents of the network solution at t = 0
    V = voltage_at_start(bus_state)
    current_ri = dq_to_ri(state.delta[1]) * [state.id[1]; state.iq[1]]
    I = current_ri[1] + im * current_ri[2]
    S = V * conj(I)

    initial = Any[
        power_quantity(inputs, "P_terminal", real(S), "MW"),
        power_quantity(inputs, "Q_terminal", imag(S), "Mvar"),
        power_quantity(inputs, "P0", generator_P0(generator), "MW"),
        power_quantity(inputs, "Q0", generator_Q0(generator), "Mvar"),
    ]
    state_values = state_initial_values(inputs, state)
    append!(initial, state_values)

    return Pair{String, Any}[
        "name" => generator_name(generator),
        "bus" => generator_bus(generator),
        "model" => string(nameof(typeof(generator))),
        "initial" => initial,
        "parameters" => generator_parameters(generator),
    ]
end

# the parameters are written in the IBR base, as the case data gives them
function ibr_parameters(ibr::IBR)
    return Any[
        quantity("Sn", ibr_Sn(ibr), "MVA"),
        quantity("D", ibr.D, "pu"),
        quantity("Kpg", ibr.Kpg, "pu"),
        quantity("Kig", ibr.Kig, "1/s"),
        quantity("Pmax", ibr.Pmax, "pu"),
        quantity("Pmin", ibr.Pmin, "pu"),
        quantity("Tp", ibr.Tp, "s"),
        quantity("Kc", ibr.Kc, "pu"),
        quantity("Kp", ibr.Kp, "pu"),
        quantity("Ki", ibr.Ki, "1/s"),
        quantity("Qmax", ibr.Qmax, "pu"),
        quantity("Qmin", ibr.Qmin, "pu"),
        quantity("Tq", ibr.Tq, "s"),
        quantity("Vfrz", ibr.Vfrz, "pu"),
        quantity("Vdip", ibr.Vdip, "pu"),
        quantity("Kqv", ibr.Kqv, "pu"),
        quantity("Iqh1", ibr.Iqh1, "pu"),
        quantity("Iql1", ibr.Iql1, "pu"),
        quantity("Imax", ibr.Imax, "pu"),
        quantity("Tg", ibr.Tg, "s"),
        quantity("Rrpwr", ibr.Rrpwr, "pu/s"),
        quantity("lv0", ibr.lv0, "pu"),
        quantity("lv1", ibr.lv1, "pu"),
    ]
end

# the battery flag and the current priority are not quantities, so they are written as keys of their own
function ibr_description(inputs::Inputs, ibr::IBR, state::IBRState)
    initial = Any[
        power_quantity(inputs, "P0", ibr_P0(ibr), "MW"),
        power_quantity(inputs, "Q0", ibr_Q0(ibr), "Mvar"),
    ]
    state_values = state_initial_values(inputs, state)
    append!(initial, state_values)

    priority = ibr_priority(ibr)
    return Pair{String, Any}[
        "name" => ibr_name(ibr),
        "bus" => ibr_bus(ibr),
        "model" => string(nameof(typeof(ibr))),
        "battery" => ibr_battery(ibr),
        "priority" => string(priority),
        "initial" => initial,
        "parameters" => ibr_parameters(ibr),
    ]
end

function load_parameters(inputs::Inputs, load::ConstantZLoad, bus::Bus)
    Y = load_constant_admittance(load_P0(load), load_Q0(load), bus_voltage(bus))
    return Any[
        power_quantity(inputs, "P0", load_P0(load), "MW"),
        power_quantity(inputs, "Q0", load_Q0(load), "Mvar"),
        quantity("G", real(Y), "pu"),
        quantity("B", imag(Y), "pu"),
    ]
end

# a constant admittance draws S = |V|^2 conj(Y), evaluated with the voltage of the network solution at t = 0
function load_initial_values(inputs::Inputs, load::ConstantZLoad, bus::Bus, bus_state::BusState)
    Y = load_constant_admittance(load_P0(load), load_Q0(load), bus_voltage(bus))
    V = voltage_at_start(bus_state)
    S = abs(V)^2 * conj(Y)
    return Any[
        power_quantity(inputs, "P", real(S), "MW"),
        power_quantity(inputs, "Q", imag(S), "Mvar"),
    ]
end

function load_description(inputs::Inputs, load::Load, bus::Bus, bus_state::BusState)
    return Pair{String, Any}[
        "name" => load_name(load),
        "bus" => load_bus(load),
        "model" => string(nameof(typeof(load))),
        "initial" => load_initial_values(inputs, load, bus, bus_state),
        "parameters" => load_parameters(inputs, load, bus),
    ]
end

# the fields that end in _idx point to the element the perturbation acts on, so new perturbation types
# are described without a change here
function perturbation_description(perturbation::Perturbation)
    targets = Pair{String, Any}[]
    parameters = Any[]
    for field in fieldnames(typeof(perturbation))
        if field == :name || field == :time
            continue
        end
        value = getfield(perturbation, field)
        field_name = string(field)
        if value isa Int && endswith(field_name, "_idx")
            target_class = chop(field_name; tail = 4)
            push!(targets, target_class => value)
        elseif value isa Float64
            push!(parameters, quantity(field_name, value, ""))
        end
    end
    return Pair{String, Any}[
        "name" => perturbation_name(perturbation),
        "type" => string(nameof(typeof(perturbation))),
        "time" => perturbation_time(perturbation),
        "targets" => targets,
        "parameters" => parameters,
    ]
end

function case_description(inputs::Inputs, results::Results)
    buses = Any[]
    for (bus, state) in zip(inputs.buses, results.bus)
        push!(buses, bus_description(inputs, bus, state))
    end

    circuits = Any[]
    for circuit in inputs.circuits
        from_state = results.bus[circuit_from_bus_idx(circuit)]
        to_state = results.bus[circuit_to_bus_idx(circuit)]
        push!(circuits, circuit_description(inputs, circuit, from_state, to_state))
    end

    generators = Any[]
    for (generator, state) in zip(inputs.generators, results.generator)
        bus_state = results.bus[generator_bus(generator)]
        push!(generators, generator_description(inputs, generator, state, bus_state))
    end

    ibrs = Any[]
    for (ibr, state) in zip(inputs.ibrs, results.ibr)
        push!(ibrs, ibr_description(inputs, ibr, state))
    end

    loads = Any[]
    for load in inputs.loads
        bus = get_bus(inputs, load_bus(load))
        bus_state = results.bus[load_bus(load)]
        push!(loads, load_description(inputs, load, bus, bus_state))
    end

    perturbations = Any[]
    for perturbation in inputs.perturbations
        push!(perturbations, perturbation_description(perturbation))
    end

    return Pair{String, Any}[
        "system" => system_description(inputs),
        "buses" => buses,
        "circuits" => circuits,
        "generators" => generators,
        "ibrs" => ibrs,
        "loads" => loads,
        "perturbations" => perturbations,
    ]
end

function write_case_description(inputs::Inputs, results::Results, output_path::String)
    description = case_description(inputs, results)
    io = open(joinpath(output_path, "case.json"), "w")
    try
        write_json(io, description)
    finally
        close(io)
    end
    return nothing
end
