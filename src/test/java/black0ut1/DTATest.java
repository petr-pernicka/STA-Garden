package black0ut1;

import black0ut1.data.DoubleMatrix;
import black0ut1.data.network.Network;
import black0ut1.dynamic.Convergence;
import black0ut1.dynamic.DynamicNetwork;
import black0ut1.dynamic.TimeDependentODM;
import black0ut1.dynamic.equilibrium.*;
import black0ut1.dynamic.loading.dnl.BasicDNL;
import black0ut1.dynamic.loading.dnl.DynamicNetworkLoading;
import black0ut1.dynamic.tdsp.DOT;
import black0ut1.io.TNTP;
import black0ut1.util.Util;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

import java.util.stream.Stream;

//@Disabled
public class DTATest {
	
	static Stream<Arguments> provideConfigurations() {
		// Network, step size, odm steps, time steps, msa steps
		return Stream.of(
				/*Arguments.of("SiouxFalls", 1, 30, 300, 100, 10)*/
				Arguments.of("ChicagoSketch", 0.1, 30, 1000, 10, 1));
	}
	
	@ParameterizedTest
	@MethodSource("provideConfigurations")
	void test(String networkName, double stepSize, int odmSteps, int timeSteps, int msaSteps, double odmScale) {
		String networkFile = "data/" + networkName + "/" + networkName + "_net.tntp";
		String odmFile = "data/" + networkName + "/" + networkName + "_trips.tntp";
		String nodeFile = "data/" + networkName + "/" + networkName + "_node.tntp";
		var pair = Util.loadData(new TNTP(), networkFile, odmFile, nodeFile);
		Network network = pair.first();
		DoubleMatrix odm = pair.second();
		
		TimeDependentODM tdodm = TimeDependentODM
				.fromStaticGaussian(odm, odmSteps)
				.scale(odmScale);
		DynamicNetwork dynamicNetwork = DynamicNetwork.fromStaticNetwork(network, tdodm, stepSize, timeSteps);
		
		StaticRouteChoice routeChoice = new StaticAONRouteChoice(network, dynamicNetwork, timeSteps);
		DynamicNetworkLoading dnl = new BasicDNL(dynamicNetwork, tdodm, stepSize, timeSteps);
		DOT tdsp = new DOT(dynamicNetwork, stepSize, timeSteps, true);
		
		Convergence convergence = new Convergence(dynamicNetwork, tdodm, stepSize, null);
		
		MOF_DUE msa = new MSA(dynamicNetwork, tdodm, routeChoice, dnl, tdsp, msaSteps, stepSize, convergence);
		long tick = System.currentTimeMillis();
		msa.run();
		long tock = System.currentTimeMillis();
//		dnl.checkDestinationInflows(true);
		System.out.println("DTA took: " + (tock - tick) + "ms");
	}
}
