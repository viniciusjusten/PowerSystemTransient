mutable struct Bus
    name::String

    generator_idx::Union{Int, Nothing} # index of the generator connected to this bus, if any
    load_idx::Union{Int, Nothing} # index of the load connected to this bus, if any
    ibr_idx::Union{Int, Nothing} # index of the IBR connected to this bus, if any

    voltage::Float64 # voltage magnitude at the power flow solution, in pu
    angle::Float64 # voltage angle at the power flow solution, in rad
    shunt_susceptance::Float64 # shunt susceptance, in pu

    # Measurements
    freq_estimation::Union{Nothing, BusFreqEstimation} # frequency estimation of the bus, nothing when it is not estimated

    # Cache
    fault_admittance::ComplexF64 # shunt admittance of an applied short circuit, zero without a fault, in pu
end

function Bus(;
    name::String,
    voltage::Float64,
    angle::Float64,
    shunt_susceptance::Float64,
    generator_idx::Union{Int, Nothing} = nothing,
    load_idx::Union{Int, Nothing} = nothing,
    ibr_idx::Union{Int, Nothing} = nothing,
    freq_estimation::Union{Nothing, BusFreqEstimation} = nothing,
)
    return Bus(
        name,
        generator_idx,
        load_idx,
        ibr_idx,
        voltage,
        angle,
        shunt_susceptance,
        freq_estimation,
        0.0 + 0.0im,
    )
end

bus_name(bus::Bus) = bus.name

bus_generator_idx(bus::Bus) = bus.generator_idx

bus_load_idx(bus::Bus) = bus.load_idx

bus_ibr_idx(bus::Bus) = bus.ibr_idx

bus_voltage(bus::Bus) = bus.voltage

bus_angle(bus::Bus) = bus.angle

bus_shunt_susceptance(bus::Bus) = bus.shunt_susceptance

bus_freq_estimation(bus::Bus) = bus.freq_estimation

has_freq_estimation(bus::Bus) = !isnothing(bus.freq_estimation)

bus_fault_admittance(bus::Bus) = bus.fault_admittance
