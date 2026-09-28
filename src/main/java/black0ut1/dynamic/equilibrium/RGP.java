package black0ut1.dynamic.equilibrium;

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
 * Reduced Gradient Projection algorithm.
 * <p>
 * Bibliography:																		  <br>
 * - (Gentile, 2016) Solving a Dynamic User Equilibrium model based on splitting rates
 * with Gradient Projection algorithms
 */
public class RGP extends MOF_DUE {
	
	protected static final double RO = 1;
	protected static final double ETA_1 = 2;
	protected static final double ETA_2 = 2 / 3.0;
	protected static final double ETA_3 = 1;
	
	protected final ScaleFactor scaleFactor = ScaleFactor.MIN_COST;
	
	public RGP(DynamicNetwork network, TimeDependentODM odm, StaticRouteChoice initialRouteChoice,
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
						if (minCost == 0 || minCost == Double.POSITIVE_INFINITY)
							continue;
						
						byte bestLinkIndex = shortestOugoingLinks.getIndex(n, t, d);
						
						double sum = 0;
						for (int j = 0; j < outgoingLinks.length; j++) {
							if (j == bestLinkIndex)
								continue;
							
							if (outgoingLinks[j] instanceof Connector)
								continue;
							
							double cost = tdsp.computeCost(t, d, outgoingLinks[j], travelTimes, costs);
							double g = scaleFactor(minCost, cost, alpha);
							double delta = (cost - minCost) / (2 * g); // positive since minCost < cost
							
							double newValue = Math.max(0, mof.getFraction(t, d, j) - delta);
							mof.setFraction(t, d, j, newValue);
							sum += newValue;
						}
						
						mof.setFraction(t, d, bestLinkIndex, 1 - sum);
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
	
	protected double scaleFactor(double minCost, double cost, double alpha) {
		double numerator = switch (scaleFactor) {
			case MIN_COST -> minCost;
			case COST -> cost;
		};
		return numerator / (RO * alpha);
	}
	
	public enum ScaleFactor {
		MIN_COST,
		COST
	}
}
