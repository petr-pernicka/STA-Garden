package black0ut1.io;

import black0ut1.data.DoubleMatrix;
import black0ut1.data.network.Network;

import java.io.BufferedReader;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Vector;

/**
 * This subclass os {@link InputOutput} loads traffic data from CSV format as it is
 * defined in the
 * <a href="https://github.com/HanZhengIntelliTransport/GMNS_Plus_Dataset/">GMNS Plus Dataset</a>.
 */
public class CSV extends InputOutput {
	
	protected final static String DELIMITER = ",";
	
	protected final static String TAIL_COLUMN = "from_node_id";
	protected final static String HEAD_COLUMN = "to_node_id";
	protected final static String CAPACITY_COLUMN = "capacity";
	protected final static String LENGTH_COLUMN = "vdf_length_mi";
	protected final static String ALPHA_COLUMN = "vdf_alpha";
	protected final static String BETA_COLUMN = "vdf_beta";
	protected final static String FREE_FLOW_TIME_COLUMN = "vdf_fftt";
	
	@Override
	protected List<Network.Edge> readEdges(String networkFile) {
		try (BufferedReader reader = Files.newBufferedReader(Paths.get(networkFile))) {
			var header = parseCsvHeader(reader);
			
			int zeroFreeFlowEdges = 0;
			
			Vector<Network.Edge> edges = new Vector<>();
			
			String line;
			while ((line = reader.readLine()) != null) {
				String[] split = line.split(DELIMITER);
				
				double freeFlow = Double.parseDouble(split[header.get(FREE_FLOW_TIME_COLUMN)]);
				if (freeFlow == 0) {
					zeroFreeFlowEdges++;
					freeFlow = 0.0001;
				}
				
				int tail = Integer.parseInt(split[header.get(TAIL_COLUMN)]) - 1;
				int head = Integer.parseInt(split[header.get(HEAD_COLUMN)]) - 1;
				double capacity = Double.parseDouble(split[header.get(CAPACITY_COLUMN)]);
				double length = Double.parseDouble(split[header.get(LENGTH_COLUMN)]);
				double alpha = Double.parseDouble(split[header.get(ALPHA_COLUMN)]);
				double beta = Double.parseDouble(split[header.get(BETA_COLUMN)]);
				edges.add(new Network.Edge(tail, head, capacity, length, freeFlow, alpha, beta));
			}
			
			if (zeroFreeFlowEdges > 0)
				System.out.println("Warning! Found " + zeroFreeFlowEdges + " edges with zero free flow.");
			
			return edges;
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
	}
	
	protected Map<String, Integer> parseCsvHeader(BufferedReader reader) throws IOException {
		String header = reader.readLine();
		String[] split = header.split(DELIMITER);
		
		HashMap<String, Integer> result = new HashMap<>();
		for (int i = 0; i < split.length; i++)
			result.put(split[i], i);
		
		return result;
	}
	
	@Override
	protected Network.Node[] readNodes(String nodeFile, int nodesNum) {
		try (BufferedReader reader = Files.newBufferedReader(Paths.get(nodeFile))) {
			Network.Node[] nodes = new Network.Node[nodesNum];
			
			HashMap<String, Integer> header = new HashMap<>();
			String[] headerLine = reader.readLine().split(",");
			for (int i = 0; i < headerLine.length; i++)
				header.put(headerLine[i], i);
			
			reader.lines()
					.map(line -> line.split(DELIMITER))
					.forEach(arr -> {
						int nodeIndex = Integer.parseInt(arr[header.get("node_id")]) - 1;
						double x = Double.parseDouble(arr[header.get("x_coord")]);
						double y = Double.parseDouble(arr[header.get("y_coord")]);
						nodes[nodeIndex] = new Network.Node(nodeIndex, x, y);
					});
			
			return nodes;
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
	}
	
	@Override
	public DoubleMatrix parseODMatrix(String odmFile) {
		try (BufferedReader reader = Files.newBufferedReader(Paths.get(odmFile))) {
			var odPairs = reader.lines()
					.skip(1)
					.map(line -> line.split(DELIMITER))
					.map(arr -> {
						int origin = Integer.parseInt(arr[0]) - 1;
						int destination = Integer.parseInt(arr[1]) - 1;
						double demand = Double.parseDouble(arr[2]);
						return new Object[]{origin, destination, demand};
					})
					.toList();
			
			int zones = odPairs.stream()
					.mapToInt(arr -> Math.max((int) arr[0], (int) arr[1]))
					.max()
					.getAsInt() + 1;
			
			DoubleMatrix ODM = new DoubleMatrix(zones);
			
			for (Object[] odPair : odPairs)
				ODM.set((int) odPair[0], (int) odPair[1], (double) odPair[2]);
			
			return ODM;
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
	}
	
	@Override
	public void writeFlows(String outputFile, Network network, double[] flows, double[] costs) {
		// TODO
	}
	
	@Override
	public void writeODmatrix(String outputFile, DoubleMatrix ODMatrix) {
		// TODO
	}
}
