package black0ut1.dynamic.loading.routing;

import black0ut1.dynamic.DynamicNetwork;

import java.util.*;

public class MixtureOutgoingFractions {
	
	public final Intersection[] mofIntersections;
	protected final DynamicNetwork network;
	protected final double[] temporaryGrid;
	
	public final int intersections;
	public final int destinations;
	public final int timeSteps;
	
	public MixtureOutgoingFractions(DynamicNetwork network, int timeSteps) {
		this.network = network;
		this.intersections = network.routedIntersections.length;
		this.destinations = network.zones.length;
		this.timeSteps = timeSteps;
		
		this.mofIntersections = new Intersection[intersections];
		for (int i = 0; i < intersections; i++)
			mofIntersections[i] = new Intersection(i);
		
		int maxJ = 0;
		for (int i = 0; i < intersections; i++)
			maxJ = Math.max(maxJ, mofIntersections[i].J);
		this.temporaryGrid = new double[destinations * timeSteps * maxJ];
	}
	
	public Intersection get(int n) {
		return mofIntersections[n];
	}
	
	public class Intersection {
		
		public final int J;
		
		public int[] offsets = new int[destinations + 1];
		public int[] indices;
		public int[] values;
		public double[] uniqueVectors;
		
		public Intersection(int n) {
			this.J = network.routedIntersections[n].outgoingLinks.length;
		}
		
		public void start() {
			Arrays.fill(temporaryGrid, 0);
		}
		
		public void setFraction(int t, int d, int j, double val) {
			temporaryGrid[t * destinations * J + d * J + j] = val;
		}
		
		public double getFraction(int t, int d, int j) {
			int i = offsets[d];
			int last = offsets[d + 1] - 1;
			
			// indices[offsets[d]] == 0 <= t, so the first period always qualifies.
			// Advance while the next period has already started.
			while (i < last && indices[i + 1] <= t)
				i++;
			
			int index = values[i];
			if (index >= 0)
				return index == j ? 1 : 0;
			else {
				int poolIndex = -(index + 1);
				return uniqueVectors[poolIndex * J + j];
			}
		}
		
		public void compress() {
			int[] grid = new int[timeSteps * destinations];
			
			int uniqueVectorsLen = 0;
			Map<double[], Integer> vectorToIndex = new TreeMap<>((o1, o2) -> {
				for (int i = 0; i < J; i++) {
					if (o1[i] < o2[i])
						return -1;
					if (o1[i] > o2[i])
						return 1;
				}
				
				return 0;
			});
			
			for (int d = 0; d < destinations; d++) {
				for (int t = 0; t < timeSteps; t++) {
					double[] vector = Arrays.copyOfRange(temporaryGrid,
							t * destinations * J + d * J, t * destinations * J + d * J + J);
					
					int singletonIndex = isSingleton(vector);
					
					if (singletonIndex >= 0)
						// If the vector is a singleton, set the grid to the index of the
						// 1 value.
						grid[t * destinations + d] = singletonIndex;
					
					else {
						// If not, set the value in grid to the index of the vector in the
						// pool of unique vectors, adding it if necessary.
						// Index i into the pool is stored in the grid as -(i + 1),
						// because non-negative integers represent singletons.
						Integer index = vectorToIndex.get(vector);
						if (index != null)
							grid[t * destinations + d] = -(index + 1);
						else {
							vectorToIndex.put(vector, uniqueVectorsLen);
							grid[t * destinations + d] = -(uniqueVectorsLen + 1);
							uniqueVectorsLen++;
						}
					}
				}
			}
			
			uniqueVectors = new double[uniqueVectorsLen * J];
			for (Map.Entry<double[], Integer> entry : vectorToIndex.entrySet()) {
				int poolIndex = entry.getValue();
				double[] vector = entry.getKey();
				System.arraycopy(vector, 0, uniqueVectors, poolIndex * J, J);
			}
			
			// Compress the grid.
			// offsets[0] = 0;
			for (int d = 0; d < destinations; d++) {
				
				int numPeriods = 1; // the number of periods for this destination
				for (int t = 1; t < timeSteps; t++)
					if (grid[t * destinations + d] != grid[(t - 1) * destinations + d])
						numPeriods++;
				
				offsets[d + 1] = offsets[d] + numPeriods;
			}
			int totalNumPeriods = offsets[destinations];
			indices = new int[totalNumPeriods];
			values = new int[totalNumPeriods];
			
			for (int d = 0; d < destinations; d++) {
				
				// values[i] is the value that lies in the grid on positions from
				// grid[indices[i] * destinations + d] to
				// grid[(indices[i + 1] - 1) * destinations + d].
				// That is, on the positions in the grid where t is in interval
				// [indices[i], indices[i + 1]) and for this destination.
				indices[offsets[d]] = 0;
				values[offsets[d]] = grid[d]; // grid[0 * destinations + d]
				int i = offsets[d] + 1;
				for (int t = 1; t < timeSteps; t++)
					if (grid[t * destinations + d] != grid[(t - 1) * destinations + d]) {
						indices[i] = t;
						values[i] = grid[t * destinations + d];
						i++;
					}
			}
		}
		
		/**
		 * Determines, whether a vector is a singleton, returning the index of the single
		 * value 1 in the vector. Otherwise, this method returns -1.
		 */
		protected int isSingleton(double[] vector) {
			for (int j = 0; j < J; j++) {
				if (vector[j] == 1)
					return j;
				if (vector[j] > 0)
					return -1;
			}
			
			return -1;
		}
	}
	
	public class Indices {
		
		protected final byte[] values = new byte[intersections * destinations * timeSteps];
		
		public byte getIndex(int n, int t, int d) {
			return values[n * timeSteps * destinations + t * destinations + d];
		}
		
		public void setIndex(int n, int t, int d, byte j) {
			values[n * timeSteps * destinations + t * destinations + d] = j;
		}
	}
	
	public class Costs {
		
		protected final double[] values = new double[intersections * destinations * timeSteps];
		
		public Costs(double value) {
			Arrays.fill(values, value);
		}
		
		public double getCost(int n, int t, int d) {
			return values[d * timeSteps * intersections + t * intersections + n];
		}
		
		public void setCost(int n, int t, int d, double cost) {
			values[d * timeSteps * intersections + t * intersections + n] = cost;
		}
	}
}
