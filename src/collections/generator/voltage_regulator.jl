abstract type VoltageRegulator end

mutable struct Static1stOrder <: VoltageRegulator
    name::String

    # Parameters
    K::Float64 # gain of the regulator and exciter, in pu/pu
    T::Float64 # time constant of the regulator and exciter, in s
    Lmn::Float64 # lower limit of the field voltage, in pu
    Lmx::Float64 # upper limit of the field voltage, in pu
end

function Static1stOrder(;
    name::String,
    K::Float64,
    T::Float64,
    Lmn::Float64,
    Lmx::Float64,
)
    return Static1stOrder(
        name,
        K,
        T,
        Lmn,
        Lmx,
    )
end

voltage_regulator_name(voltage_regulator::VoltageRegulator) = voltage_regulator.name

voltage_regulator_K(voltage_regulator::Static1stOrder) = voltage_regulator.K

voltage_regulator_T(voltage_regulator::Static1stOrder) = voltage_regulator.T

voltage_regulator_Lmn(voltage_regulator::Static1stOrder) = voltage_regulator.Lmn

voltage_regulator_Lmx(voltage_regulator::Static1stOrder) = voltage_regulator.Lmx
