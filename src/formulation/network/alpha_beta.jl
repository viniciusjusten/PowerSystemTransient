# Companion models of the alpha beta network, written in the stationary reference frame. The rest of the network
# solution is shared with the other EMT networks, in emt.jl

# The voltages and currents are stored as phasors in the synchronous reference frame, which turns at wb with respect
# to the stationary frame, so a phasor X at the time t is the space vector X e^(j wb t)
function frame_rotation(::NetworkAlphaBeta, inputs::Inputs, ts::Int)
    wb = 2 * pi * fb(inputs)
    t = time_at(inputs, ts)
    return cis(wb * t)
end
