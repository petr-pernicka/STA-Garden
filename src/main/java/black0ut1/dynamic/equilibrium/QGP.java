package black0ut1.dynamic.equilibrium;

import black0ut1.data.BitSet32;
import black0ut1.dynamic.Convergence;
import black0ut1.dynamic.DynamicNetwork;
import black0ut1.dynamic.TimeDependentODM;
import black0ut1.dynamic.loading.dnl.DynamicNetworkLoading;
import black0ut1.dynamic.loading.link.Connector;
import black0ut1.dynamic.loading.link.Link;
import black0ut1.dynamic.loading.routing.MixtureOutgoingFractions;
import black0ut1.dynamic.tdsp.DOT;
import black0ut1.util.DynamicUtils;

/**
 * Quasi-Gradient Projection algorithm.
 * <p>
 * Bibliography:																		  <br>
 * - (Gentile, 2016) Solving a Dynamic User Equilibrium model based on splitting rates
 * with Gradient Projection algorithms
 */
public class QGP extends RGP {
	
	public QGP(DynamicNetwork network, TimeDependentODM odm, StaticRouteChoice initialRouteChoice,
	           DynamicNetworkLoading dnl, DOT tdsp, int maxIterations, double stepSize, Convergence convergence) {
		super(network, odm, initialRouteChoice, dnl, tdsp, maxIterations, stepSize, convergence);
	}
	
	public void run() {
		var mfs = initialRouteChoice.computeInitialMixtureFractions();
		dnl.setTurningFractions(mfs);
		dnl.loadNetwork();
//		dnl.checkDestinationInflows(false);
		
		double[][] travelTimes = new double[network.links.length][];
		for (int i = 0; i < network.links.length; i++)
			travelTimes[i] = DynamicUtils.computeTravelTime(network.links[i], stepSize);
		
		var pair = tdsp.shortestPaths(mfs, travelTimes);
		MixtureOutgoingFractions.Costs costs = pair.first();
		MixtureOutgoingFractions.Indices shortestOugoingLinks = pair.second();
		
		double[] criterions = convergence.computeAll(shortestOugoingLinks);
		System.out.println("[DUE] TSTT: " + criterions[0]);
		System.out.println("[DUE] SPTT: " + criterions[1]);
		System.out.println("[DUE] AEC:  " + criterions[2]);
		System.out.println("[DUE] RG:   " + criterions[3]);
		
		int eta_bad = 0;
		for (int i = 0; i < maxIterations; i++) {
			System.out.println("[DUE] Iteration: " + i);
			
			double alpha = Math.pow(ETA_1 / (ETA_1 + eta_bad), ETA_2);
			for (int n = 0; n < mfs.intersections; n++) {
				Link[] outgoingLinks = network.routedIntersections[n].outgoingLinks;
				MixtureOutgoingFractions.Intersection mof = mfs.get(n);
				mof.start();
				
				for (int t = 0; t < mfs.timeSteps; t++)
					for (int d = 0; d < network.zones.length; d++) {
						double minCost = costs.getCost(n, t, d);
						if (minCost == 0)
							continue;
						
						double[] costs1 = new double[outgoingLinks.length];
						double[] gs = new double[outgoingLinks.length];
						for (int j = 0; j < outgoingLinks.length; j++) {
							if (outgoingLinks[j] instanceof Connector)
								continue;
							
							costs1[j] = tdsp.computeCost(t, d, outgoingLinks[j], travelTimes, costs);
							gs[j] = scaleFactor(minCost, costs1[j], alpha);
						}
						
						// hack that avoids NaNs, which occur on line
						//    ? (avgCost - costs1[j]) / gs[j]
						// and still gives correct results
						for (int j = 0; j < outgoingLinks.length; j++)
							costs1[j] = Math.min(costs1[j], Double.MAX_VALUE);
						
						BitSet32 B = BitSet32.filled(outgoingLinks.length);
						for (int j = 0; j < outgoingLinks.length; j++)
							if (outgoingLinks[j] instanceof Connector)
								B.clear(j);
						
						double[] delta = new double[outgoingLinks.length];
						while (true) {
							
							// Compute the weighted average cost of set B
							double numerator = 0;
							double denominator = 0;
							for (int j = 0; j < outgoingLinks.length; j++)
								if (B.get(j)) {
									numerator += costs1[j] / gs[j];
									denominator += 1 / gs[j];
								}
							
							double avgCost = numerator / denominator;
							
							boolean eliminated = false;
							for (int j = 0; j < outgoingLinks.length; j++) {
								delta[j] = B.get(j)
										? (avgCost - costs1[j]) / gs[j]
										: 0;
								
								if (mof.getFraction(t, d, j) == 0 && delta[j] < 0) {
									B.clear(j);
									eliminated = true;
								}
							}
							
							if (!eliminated)
								break;
						}
						
						// Compute parameter beta
						double beta = 1;
						for (int j = 0; j < outgoingLinks.length; j++)
							if (B.get(j) && delta[j] < 0)
								beta = Math.min(beta, -mof.getFraction(t, d, j) / delta[j]);
						
						for (int j = 0; j < outgoingLinks.length; j++) {
							if (outgoingLinks[j] instanceof Connector)
								continue;
							
							double newValue = Math.max(0, mof.getFraction(t, d, j) + beta * delta[j]);
							mof.setFraction(t, d, j, newValue);
						}
					}
				mof.compress();
			}
			
			dnl.loadNetwork();
			
			for (int j = 0; j < network.links.length; j++)
				travelTimes[j] = DynamicUtils.computeTravelTime(network.links[j], stepSize);
			
			pair = tdsp.shortestPaths(mfs, travelTimes);
			costs = pair.first();
			shortestOugoingLinks = pair.second();
			
			double[] newCriterions = convergence.computeAll(shortestOugoingLinks);
			System.out.println("[DUE] TSTT: " + newCriterions[0]);
			System.out.println("[DUE] SPTT: " + newCriterions[1]);
			System.out.println("[DUE] AEC:  " + newCriterions[2]);
			System.out.println("[DUE] RG:   " + newCriterions[3]);
			
			// If new AEC is larger than RO * old AEC, eta_bad is incremented
			if (newCriterions[2] >= ETA_3 * criterions[2])
				eta_bad++;
			
			criterions = newCriterions;
		}
		
		dnl.checkDestinationInflows(false);
	}
}
