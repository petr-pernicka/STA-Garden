package black0ut1.dynamic;

import black0ut1.dynamic.loading.link.Link;
import black0ut1.dynamic.loading.routing.MixtureOutgoingFractions;
import black0ut1.util.DynamicUtils;

import java.util.function.Consumer;

public class  Convergence {
	
	protected final DynamicNetwork network;
	protected final TimeDependentODM odm;
	protected final double stepSize;
	protected final int timeSteps;
	protected final double totalDemand;
	
	protected final Consumer<double[]> callback;
	
	public Convergence(DynamicNetwork network, TimeDependentODM odm,
					   double stepSize, int timeSteps, Consumer<double[]> callback) {
		this.network = network;
		this.odm = odm;
		this.stepSize = stepSize;
		this.timeSteps = timeSteps;
		this.callback = callback;
		
		double totalDemand1 = 0;
		for (int origin = 0; origin < odm.zones; origin++)
			for (int destination = 0; destination < odm.zones; destination++)
				for (int t = 0; t < odm.timeSteps; t++)
					totalDemand1 += odm.getDemand(origin, destination, t);
		this.totalDemand = totalDemand1;
	}
	
	public double totalSystemTravelTime() {
		double tstt = 0;
		
		for (Link link : network.links)
			// The area between the cumulative curves, i.e. the number of vehicles on the
			// link integrated over time.
			for (int t = 0; t < link.cumulativeInflow.length - 1; t++) {
				double n1 = link.cumulativeInflow[t] - link.cumulativeOutflow[t];
				double n2 = link.cumulativeInflow[t + 1] - link.cumulativeOutflow[t + 1];
				tstt += (n1 + n2) / 2;
			}
		
		return tstt * stepSize;
	}
	
	public double shortestPathTravelTime(MixtureOutgoingFractions.Indices indices) {
		// Since the DOT algorithm gives non-exact costs, the computation of SPTT the way
		// that is commented out is approximate and often larger than TSTT, leading to
		// negative RG and AEC. The used way retraces the exact cost along the path given
		// by the DOT as the shortest path. Keep in mind that this path is not necessarily
		// the shortest path as it is also a subject of errors made by the time
		// discretization used in DOT. The used method thus gives more accurate SPTT but
		// still not the true value.
//		double sptt = 0;
//
//		for (int origin = 0; origin < odm.zones; origin++)
//			for (int destination = 0; destination < odm.zones; destination++)
//				for (int t = 0; t < odm.timeSteps; t++) {
//					double demand = odm.getDemand(origin, destination, t);
//					if (demand > 0) // to avoid NaNs
//						sptt += costs.getCost(origin, t, destination) * demand;
//				}
//
//		return sptt;
		
		double sptt = 0;
		
		for (int origin = 0; origin < odm.zones; origin++)
			for (int destination = 0; destination < odm.zones; destination++)
				for (int t = 0; t < odm.timeSteps; t++) {
					double demand = odm.getDemand(origin, destination, t);
					if (demand > 0) // to avoid NaNs
						sptt += demand * retrace(origin, destination, t, indices);
				}
		
		return sptt;
	}
	
	private double retrace(int origin, int destination, int t, MixtureOutgoingFractions.Indices indices) {
		double cost = 0; // regular (non-normalized) time
		int tail = origin;
		double time = t; // normalized time
		
		while (tail != destination) {
			byte j = (time >= timeSteps - 1)
					? indices.getIndex(tail, timeSteps - 1, destination)
					: indices.getIndex(tail, (int) time, destination);
			Link link = network.routedIntersections[tail].outgoingLinks[j];
			
			double linkTravelTime = DynamicUtils.computeTravelTime(t, link, stepSize);
			cost += linkTravelTime;
			time += linkTravelTime / stepSize;
			tail = link.head.index;
		}
		
		return Math.min(cost, (timeSteps - t) * stepSize);
	}
	
	public double averageExcessCost(double tstt, double sptt) {
		return (tstt - sptt) / totalDemand;
	}
	
	public double relativeGap(double tstt, double sptt) {
		return (tstt - sptt) / sptt;
	}
	
	public double[] computeAll(MixtureOutgoingFractions.Indices indices) {
		double ttst = totalSystemTravelTime();
		double sptt = shortestPathTravelTime(indices);
		double[] all = new double[] {
				ttst,
				sptt,
				averageExcessCost(ttst, sptt),
				relativeGap(ttst, sptt)
		};
		
		if (callback != null)
			callback.accept(all);
		
		return all;
	}
}
