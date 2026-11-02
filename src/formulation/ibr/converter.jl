# Converter, not implemented yet. Ip and Iq will be states that follow Ip_cmd and Iq_cmd of the current logic with the
# lag 1 / (1 + s Tg). Two decisions are already taken for it.
#
# The voltage dip status, Vt < Vdip, which switches iq_inj on in the current logic, will be decided from the values at
# ts and kept for the whole step, as in yp_frozen and yq_frozen. iq_inj jumps from zero to Kqv * (V0 - Vdip) when Vt
# crosses Vdip, and deciding it at ts + 1 could make the corrector alternate between two values when Vt is near Vdip.
#
# The derivative of Ip will be limited from above by an up-ramp rate Rrpwr, as in WECC REGC_A, a new field of the IBR.
# It shapes the recovery of the active power after a fault, which a rate limit on Pref would not do, since
# Ip_cmd = Pref / Vt comes back together with the voltage after the clearing.
