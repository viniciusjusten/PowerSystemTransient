# Elements of the companion models of the networks with dynamics. They keep only the physical data, so the same
# elements serve every formulation, and each formulation writes its own companion equations for them. The list of
# elements is rebuilt from the circuits, buses and loads at each perturbation, so an element is never changed in place
abstract type CompanionElement end

# Series resistance and inductance between two buses. It is the series branch of a circuit or an inductive shunt when
# to_bus_idx is 0, which stands for the ground
struct Inductor <: CompanionElement
    name::String

    from_bus_idx::Int # index of the bus at the sending end
    to_bus_idx::Int # index of the bus at the receiving end, 0 for the ground

    resistance::Float64 # series resistance, in pu
    reactance::Float64 # reactance at the base frequency, in pu
    tap_ratio::Float64 # transformer tap ratio applied to the sending end voltage, 1 without a transformer

    in_service::Bool # indicates if the element is connected
end

element_from_bus_idx(element::CompanionElement) = element.from_bus_idx

element_to_bus_idx(element::CompanionElement) = element.to_bus_idx

element_in_service(element::CompanionElement) = element.in_service

function Inductor(;
    name::String,
    from_bus_idx::Int,
    to_bus_idx::Int,
    resistance::Float64,
    reactance::Float64,
    tap_ratio::Float64 = 1.0,
    in_service::Bool = true,
)
    return Inductor(name, from_bus_idx, to_bus_idx, resistance, reactance, tap_ratio, in_service)
end

inductor_resistance(inductor::Inductor) = inductor.resistance

inductor_reactance(inductor::Inductor) = inductor.reactance

inductor_tap_ratio(inductor::Inductor) = inductor.tap_ratio

# only an Inductor can be the series branch of a transformer, so the other elements have no tap
element_tap_ratio(::CompanionElement) = 1.0

element_tap_ratio(element::Inductor) = inductor_tap_ratio(element)

# Capacitance between two buses, a shunt when to_bus_idx is 0. A conductance in parallel is a separate Resistor
struct Capacitor <: CompanionElement
    name::String

    from_bus_idx::Int # index of the bus at the sending end
    to_bus_idx::Int # index of the bus at the receiving end, 0 for the ground

    susceptance::Float64 # susceptance at the base frequency, in pu

    in_service::Bool # indicates if the element is connected
end

function Capacitor(;
    name::String,
    from_bus_idx::Int,
    to_bus_idx::Int,
    susceptance::Float64,
    in_service::Bool = true,
)
    return Capacitor(name, from_bus_idx, to_bus_idx, susceptance, in_service)
end

capacitor_susceptance(capacitor::Capacitor) = capacitor.susceptance

# Resistance between two buses, a shunt when to_bus_idx is 0. It has no state
struct Resistor <: CompanionElement
    name::String

    from_bus_idx::Int # index of the bus at the sending end
    to_bus_idx::Int # index of the bus at the receiving end, 0 for the ground

    resistance::Float64 # resistance, in pu

    in_service::Bool # indicates if the element is connected
end

function Resistor(;
    name::String,
    from_bus_idx::Int,
    to_bus_idx::Int,
    resistance::Float64,
    in_service::Bool = true,
)
    return Resistor(name, from_bus_idx, to_bus_idx, resistance, in_service)
end

resistor_resistance(resistor::Resistor) = resistor.resistance
