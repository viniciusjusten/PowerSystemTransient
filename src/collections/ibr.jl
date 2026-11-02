
# Current that keeps its reference when Ip and Iq together reach Imax, while the other one is limited by what is left,
# sqrt(Imax^2 - I^2). The values are the opposite of PQFLAG of WECC REEC_B, where 0 is the Q priority
@enumx CurrentPriority begin
    P = 0
    Q = 1
end

# Grid following IBR simplified from WECC REPC_A, REEC_B and REGC_A, keeping the plant control PIs so the steady state
# error of the active power and of the voltage droop characteristic is zero. Ip is in phase with the terminal voltage
# and Iq > 0 injects reactive power, so P = Vt * Ip and Q = Vt * Iq. The frequency comes from the frequency estimation
# of the bus the IBR is connected to, which takes the place of the PLL of the reference model
mutable struct IBR
    name::String
    bus::Int # index of the bus the IBR is connected to

    # Ratings
    Sn::Float64 # rated apparent power, in MVA
    battery::Bool # indicates if the IBR is a battery, which allows negative active current

    # Plant control, active power
    D::Float64 # frequency droop gain, 1 / R, in pu
    Kpg::Float64 # proportional gain of the active power PI, in pu/pu
    Kig::Float64 # integral gain of the active power PI, in 1/s
    Pmax::Float64 # upper limit of the active power PI, in pu
    Pmin::Float64 # lower limit of the active power PI, in pu
    Tp::Float64 # time constant of the active power reference, in s

    # Plant control, voltage
    Kc::Float64 # reactive droop of the voltage control, in pu/pu
    Kp::Float64 # proportional gain of the voltage PI, in pu/pu
    Ki::Float64 # integral gain of the voltage PI, in 1/s
    Qmax::Float64 # upper limit of the voltage PI, in pu
    Qmin::Float64 # lower limit of the voltage PI, in pu
    Tq::Float64 # time constant of the reactive power reference, in s
    Vfrz::Float64 # terminal voltage below which the voltage PI integrator freezes, in pu

    # Current logic
    Vdip::Float64 # terminal voltage below which the reactive current injection acts and the active power PI integrator freezes, in pu
    Kqv::Float64 # gain of the reactive current injection, in pu/pu
    Iqh1::Float64 # upper limit of the reactive current injection, in pu
    Iql1::Float64 # lower limit of the reactive current injection, in pu
    Imax::Float64 # maximum converter current, in pu
    priority::CurrentPriority.T # current that keeps its reference at the Imax limit, P or Q

    # Converter
    Tg::Float64 # time constant of the current controls, in s
    lv0::Float64 # terminal voltage below which the low voltage gain of the active current is zero, in pu
    lv1::Float64 # terminal voltage above which the low voltage gain of the active current is one, in pu

    # Power flow solution
    P0::Float64 # active power at the terminal, in pu
    Q0::Float64 # reactive power at the terminal, in pu
end

function IBR(;
    name::String,
    bus::Int,
    Sn::Float64,
    battery::Bool,
    D::Float64,
    Kpg::Float64,
    Kig::Float64,
    Pmax::Float64,
    Pmin::Float64,
    Tp::Float64,
    Kc::Float64,
    Kp::Float64,
    Ki::Float64,
    Qmax::Float64,
    Qmin::Float64,
    Tq::Float64,
    Vfrz::Float64,
    Vdip::Float64,
    Kqv::Float64,
    Iqh1::Float64,
    Iql1::Float64,
    Imax::Float64,
    priority::CurrentPriority.T,
    Tg::Float64,
    lv0::Float64,
    lv1::Float64,
    P0::Float64,
    Q0::Float64,
)
    return IBR(
        name,
        bus,
        Sn,
        battery,
        D,
        Kpg,
        Kig,
        Pmax,
        Pmin,
        Tp,
        Kc,
        Kp,
        Ki,
        Qmax,
        Qmin,
        Tq,
        Vfrz,
        Vdip,
        Kqv,
        Iqh1,
        Iql1,
        Imax,
        priority,
        Tg,
        lv0,
        lv1,
        P0,
        Q0,
    )
end

ibr_name(ibr::IBR) = ibr.name

ibr_bus(ibr::IBR) = ibr.bus

ibr_Sn(ibr::IBR) = ibr.Sn

ibr_battery(ibr::IBR) = ibr.battery

# The powers, currents and gains are given in the IBR base and returned in the system base. Powers and currents scale
# by Sn / Sb, since the voltage base is the same. The gains scale like the ratio of their output to their input, so
# D, Kp, Ki and Kqv, which turn a frequency or a voltage into a power or a current, scale by Sn / Sb, Kc, which turns
# a reactive power into a voltage, scales by Sb / Sn, and Kpg and Kig, from power to power, do not change
ibr_D(inputs, ibr::IBR) = ibr.D * ibr_Sn(ibr) / Sb(inputs)

ibr_Kpg(ibr::IBR) = ibr.Kpg

ibr_Kig(ibr::IBR) = ibr.Kig

ibr_Pmax(inputs, ibr::IBR) = ibr.Pmax * ibr_Sn(ibr) / Sb(inputs)

ibr_Pmin(inputs, ibr::IBR) = ibr.Pmin * ibr_Sn(ibr) / Sb(inputs)

ibr_Tp(ibr::IBR) = ibr.Tp

ibr_Kc(inputs, ibr::IBR) = ibr.Kc * Sb(inputs) / ibr_Sn(ibr)

ibr_Kp(inputs, ibr::IBR) = ibr.Kp * ibr_Sn(ibr) / Sb(inputs)

ibr_Ki(inputs, ibr::IBR) = ibr.Ki * ibr_Sn(ibr) / Sb(inputs)

ibr_Qmax(inputs, ibr::IBR) = ibr.Qmax * ibr_Sn(ibr) / Sb(inputs)

ibr_Qmin(inputs, ibr::IBR) = ibr.Qmin * ibr_Sn(ibr) / Sb(inputs)

ibr_Tq(ibr::IBR) = ibr.Tq

ibr_Vfrz(ibr::IBR) = ibr.Vfrz

ibr_Vdip(ibr::IBR) = ibr.Vdip

ibr_Kqv(inputs, ibr::IBR) = ibr.Kqv * ibr_Sn(ibr) / Sb(inputs)

ibr_Iqh1(inputs, ibr::IBR) = ibr.Iqh1 * ibr_Sn(ibr) / Sb(inputs)

ibr_Iql1(inputs, ibr::IBR) = ibr.Iql1 * ibr_Sn(ibr) / Sb(inputs)

ibr_Imax(inputs, ibr::IBR) = ibr.Imax * ibr_Sn(ibr) / Sb(inputs)

ibr_priority(ibr::IBR) = ibr.priority

ibr_Tg(ibr::IBR) = ibr.Tg

ibr_lv0(ibr::IBR) = ibr.lv0

ibr_lv1(ibr::IBR) = ibr.lv1

ibr_P0(ibr::IBR) = ibr.P0

ibr_Q0(ibr::IBR) = ibr.Q0
