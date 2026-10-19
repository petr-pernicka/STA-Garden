package black0ut1;

import black0ut1.dynamic.DynamicNetwork;
import black0ut1.dynamic.TimeDependentODM;
import black0ut1.dynamic.equilibrium.StaticAONRouteChoice;
import black0ut1.dynamic.equilibrium.StaticRouteChoice;
import black0ut1.dynamic.loading.dnl.BasicDNL;
import black0ut1.dynamic.loading.dnl.DynamicNetworkLoading;
import black0ut1.dynamic.loading.routing.MixtureFlow;
import black0ut1.io.CSV;
import black0ut1.util.Util;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

import java.util.stream.Stream;

@Disabled
public class DNLTest {
	
	static Stream<Arguments> provideConfigurations() {
		// Network, step size, odm steps, time steps
		return Stream.of(
				Arguments.of("17_Sioux_Falls", 1, 30, 60),
				Arguments.of("14_Chicago_Sketch", 0.1, 30, 350)
				/*Arguments.of("BerlinCenter", 1, 30, 630)*/
		);
	}
	
	@ParameterizedTest
	@MethodSource("provideConfigurations")
	void test(String networkName, double stepSize, int odmSteps, int timeSteps) {
		String networkFile = "data/" + networkName + "/link.csv";
		String odmFile = "data/" + networkName + "/demand.csv";
		String nodeFile = "data/" + networkName + "/node.csv";
		
		MixtureFlow.PRECISION = 0;
		var pair = Util.loadData(new CSV(), networkFile, odmFile, nodeFile);
		
		TimeDependentODM odm = TimeDependentODM.fromStaticGaussian(pair.second(), odmSteps);
		DynamicNetwork network = DynamicNetwork.fromStaticNetwork(pair.first(), odm, stepSize, timeSteps);
		
		DynamicNetworkLoading dnl = new BasicDNL(network, odm, stepSize, timeSteps);
		StaticRouteChoice routeChoice = new StaticAONRouteChoice(pair.first(), network, timeSteps);
		var initialRouteChoice = routeChoice.computeInitialMixtureFractions();
		dnl.setTurningFractions(initialRouteChoice);
		
		long tick = System.currentTimeMillis();
		dnl.loadNetwork();
		long tock = System.currentTimeMillis();
		System.out.println((tock - tick) + "ms");
		
		dnl.checkDestinationInflows(false);
	}
}
