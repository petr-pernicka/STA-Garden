package black0ut1.dynamic.equilibrium;

import black0ut1.dynamic.Convergence;
import black0ut1.dynamic.DynamicNetwork;
import black0ut1.dynamic.TimeDependentODM;
import black0ut1.dynamic.loading.dnl.DynamicNetworkLoading;
import black0ut1.dynamic.loading.routing.MixtureOutgoingFractions;
import black0ut1.dynamic.tdsp.DOT;
import black0ut1.util.DynamicUtils;

/**
 * Method of Successive Averages algorithm.
 */
public class MSA extends MOF_DUE {
	
	public MSA(DynamicNetwork network, TimeDependentODM odm, StaticRouteChoice initialRouteChoice,
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
		
		double[] criterions = convergence.computeAll(costs);
		System.out.println("[DUE] TSTT: " + criterions[0]);
		System.out.println("[DUE] SPTT: " + criterions[1]);
		System.out.println("[DUE] AEC:  " + criterions[2]);
		System.out.println("[DUE] RG:   " + criterions[3]);

		
		for (int i = 0; i < maxIterations; i++) {
			System.out.println("[DUE] Iteration: " + i);
			
			double lambda = 1.0 / (i + 2);
			
			long dueTick = System.currentTimeMillis();
			for (int n = 0; n < mfs.intersections; n++) {
				MixtureOutgoingFractions.Intersection mof = mfs.get(n);
				mof.start();
				
				for (int t = 0; t < mfs.timeSteps; t++)
					for (int d = 0; d < network.zones.length; d++)
						for (int j = 0; j < network.routedIntersections[n].outgoingLinks.length; j++) {
							double fraction = mof.getFraction(t, d, j);
							
							double newFraction = (shortestOugoingLinks.getIndex(n, t, d) == j)
									? (1 - lambda) * fraction + lambda
									: (1 - lambda) * fraction;
							
							mof.setFraction(t, d, j, newFraction);
						}
				mof.compress();
			}
			long dueTock = System.currentTimeMillis();
			
			long dnlTick = System.currentTimeMillis();
			dnl.loadNetwork();
			long dnlTock = System.currentTimeMillis();
			
			for (int j = 0; j < network.links.length; j++)
				travelTimes[j] = DynamicUtils.computeTravelTime(network.links[j], stepSize);
			
			long tdspTick = System.currentTimeMillis();
			pair = tdsp.shortestPaths(mfs, travelTimes);
			long tdspTock = System.currentTimeMillis();
			
			costs = pair.first();
			shortestOugoingLinks = pair.second();
			
			criterions = convergence.computeAll(costs);
			System.out.println("[DUE] TSTT: " + criterions[0]);
			System.out.println("[DUE] SPTT: " + criterions[1]);
			System.out.println("[DUE] AEC:  " + criterions[2]);
			System.out.println("[DUE] RG:   " + criterions[3]);
			System.out.println("DUE took:  " + (dueTock - dueTick) / 1000 + "s");
			System.out.println("DNL took:  " + (dnlTock - dnlTick) / 1000 + "s");
			System.out.println("TDSP took: " + (tdspTock - tdspTick) / 1000 + "s");
		}
		
//		dnl.checkDestinationInflows(false);
	}
}
