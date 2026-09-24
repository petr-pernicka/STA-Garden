package black0ut1.dynamic.equilibrium;

import black0ut1.data.network.Network;
import black0ut1.dynamic.DynamicNetwork;
import black0ut1.dynamic.loading.routing.MixtureOutgoingFractions;
import black0ut1.dynamic.loading.node.Intersection;
import black0ut1.util.SSSP;

public class StaticAONRouteChoice implements StaticRouteChoice {
	
	protected final Network network;
	protected final DynamicNetwork dNetwork;
	protected final int timeSteps;
	
	public StaticAONRouteChoice(Network network, DynamicNetwork dNetwork, int timeSteps) {
		this.network = network;
		this.dNetwork = dNetwork;
		this.timeSteps = timeSteps;
	}
	
	public MixtureOutgoingFractions computeInitialMixtureFractions() {
		MixtureOutgoingFractions result = new MixtureOutgoingFractions(dNetwork, timeSteps);
		
		double[] costs = new double[network.edges];
		for (int i = 0; i < network.edges; i++)
			costs[i] = network.getEdges()[i].freeFlow;
		
		Network.Edge[][] successorTrees = new Network.Edge[network.zones][];
		for (int d = 0; d < network.zones; d++)
			successorTrees[d] = SSSP.dijkstraDest(network, d, costs).first();
		
		// create first mixture fraction for each node
		for (int node = 0; node < network.nodes; node++)
			createNodeFractions(result.get(node), successorTrees, node);
		
		return result;
	}
	
	protected void createNodeFractions(MixtureOutgoingFractions.Intersection mof, Network.Edge[][] successorTrees, int node1) {
		mof.start();
		
		Intersection node = dNetwork.routedIntersections[node1];
		
		// compute fractions for a destination
		for (int destination = 0; destination < network.zones; destination++) {
			
			int J = -1;
			if (node1 == destination) {
				J = 0;
			} else {
				for (int j = 0; j < node.outgoingLinks.length; j++) {
					if (node1 < network.zones && j == 0)
						continue; // skip destination connector
					
					int index = node.outgoingLinks[j].index;
					if (successorTrees[destination][node1].index == index) {
						J = j;
						break;
					}
				}
			}
			
			for (int t = 0; t < timeSteps; t++)
				mof.setFraction(t, destination, J, 1);
		}
		
		mof.compress();
	}
}
