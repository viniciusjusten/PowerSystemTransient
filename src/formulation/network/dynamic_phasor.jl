function initialize_network!(network::NetworkDP, inputs::Inputs, results::Results)
    network.elements = companion_elements(inputs)
    return nothing
end
