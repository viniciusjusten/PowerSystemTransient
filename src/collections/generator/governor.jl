abstract type Governor end

mutable struct Droop1stOrder <: Governor
    name::String

    # Parameters
    R::Float64 # droop, in pu of the machine base
    Tc::Float64 # regulator and valve time constant, in s
    Tr::Float64 # reheater time constant, in s
    omega_ref::Float64 # reference frequency, in Hz
end

function Droop1stOrder(;
    name::String,
    R::Float64,
    Tc::Float64,
    Tr::Float64,
    omega_ref::Float64,
)
    return Droop1stOrder(
        name,
        R,
        Tc,
        Tr,
        omega_ref,
    )
end

governor_name(governor::Governor) = governor.name

governor_Tc(governor::Droop1stOrder) = governor.Tc

governor_Tr(governor::Droop1stOrder) = governor.Tr

# The droop is given in the machine base and returned in the system base. It relates a frequency deviation to a
# power deviation, dP = -domega / R, so it scales like an impedance
governor_R(inputs, generator, governor::Droop1stOrder) = governor.R * Sb(inputs) / generator_Sn(generator)

governor_omega_ref(inputs, governor::Droop1stOrder) = governor.omega_ref / fb(inputs)
