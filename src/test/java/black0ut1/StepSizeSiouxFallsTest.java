package black0ut1;

import black0ut1.data.DoubleMatrix;
import black0ut1.data.network.Network;
import black0ut1.dynamic.Convergence;
import black0ut1.dynamic.DynamicNetwork;
import black0ut1.dynamic.TimeDependentODM;
import black0ut1.dynamic.equilibrium.MSA;
import black0ut1.dynamic.equilibrium.StaticAONRouteChoice;
import black0ut1.dynamic.equilibrium.StaticRouteChoice;
import black0ut1.dynamic.loading.dnl.DynamicNetworkLoading;
import black0ut1.dynamic.loading.dnl.ILTM_DNL;
import black0ut1.dynamic.tdsp.CDOT;
import black0ut1.dynamic.tdsp.DOT;
import black0ut1.io.TNTP;
import black0ut1.util.Util;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.LogAxis;
import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.axis.NumberTickUnit;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.XYLineAndShapeRenderer;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

import javax.swing.*;
import java.awt.*;
import java.util.concurrent.atomic.AtomicInteger;

@Disabled
public class StepSizeSiouxFallsTest {
	
	static final String networkName = "SiouxFalls";
	static final int MSA_STEPS = 100;
	static final int ODM_STEPS = 30;
	static final int TIME_STEPS_DEFAULT = 110;
	
	static Network network;
	static DoubleMatrix odm;
	
	static XYSeriesCollection dataset = new XYSeriesCollection();
	
	@BeforeAll
	static void beforeAll() {
		String networkFile = "data/" + networkName + "/" + networkName + "_net.tntp";
		String odmFile = "data/" + networkName + "/" + networkName + "_trips.tntp";
		String nodeFile = "data/" + networkName + "/" + networkName + "_node.tntp";
		var pair = Util.loadData(new TNTP(), networkFile, odmFile, nodeFile);
		network = pair.first();
		odm = pair.second();
		
		LogAxis yAxis = new LogAxis("AEC");
		yAxis.setTickUnit(new NumberTickUnit(1));
		NumberAxis xAxis = new NumberAxis("Iteration");
		XYLineAndShapeRenderer renderer = new XYLineAndShapeRenderer(true, false);
		XYPlot plot = new XYPlot(dataset, xAxis, yAxis, renderer);
		JFreeChart chart = new JFreeChart("Convergence of DTA by step size - Sioux Falls", JFreeChart.DEFAULT_TITLE_FONT, plot, true);
		ChartPanel chartPanel = new ChartPanel(chart);
		chartPanel.setPreferredSize(new Dimension(800, 600));
		
		JFrame frame = new JFrame();
		frame.setContentPane(chartPanel);
		frame.pack();
		frame.setLocationRelativeTo(null);
		frame.setVisible(true);
	}
	
	@AfterAll
	static void afterAll() throws InterruptedException {
		Thread.currentThread().join();
	}
	
	@ParameterizedTest
	@ValueSource(doubles = { 2, 1, 0.5, 0.1, 0.05, 0.01 })
	void test(double stepSize) {
		int timeSteps = (int) (TIME_STEPS_DEFAULT / stepSize);
		
		TimeDependentODM tdodm = TimeDependentODM
				.fromStaticGaussian(odm, ODM_STEPS)
				.scale(10);
		DynamicNetwork dynamicNetwork = DynamicNetwork.fromStaticNetwork(network, tdodm, stepSize, timeSteps);
		
		StaticRouteChoice routeChoice = new StaticAONRouteChoice(network, dynamicNetwork, timeSteps);
		DynamicNetworkLoading dnl = new ILTM_DNL(dynamicNetwork, tdodm, stepSize, timeSteps, 1e-8);
		DOT tdsp = new CDOT(dynamicNetwork, stepSize, timeSteps, false);
		
		XYSeries series = new XYSeries(stepSize);
		dataset.addSeries(series);
		AtomicInteger i = new AtomicInteger();
		Convergence convergence = new Convergence(dynamicNetwork, tdodm, stepSize,
				doubles -> series.add(i.getAndIncrement(), doubles[2])
		);
		
		MSA msa = new MSA(dynamicNetwork, tdodm, routeChoice, dnl, tdsp, MSA_STEPS, stepSize, convergence);
		long tick = System.currentTimeMillis();
		msa.run();
		long tock = System.currentTimeMillis();
		System.out.println("DTA took: " + (tock - tick) + "ms");
	}
}
