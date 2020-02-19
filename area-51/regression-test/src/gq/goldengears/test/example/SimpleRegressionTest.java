package gq.goldengears.test.example;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JFrame;
import javax.swing.SwingUtilities;
import javax.swing.WindowConstants;

import org.apache.commons.math3.stat.regression.SimpleRegression;
import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.XYPlot;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

import com.orsoncharts.*;

public class SimpleRegressionTest extends JFrame {
	
	private static SimpleRegression regression;
	private static List<Double> vuforiaReadings;
	private static List<Double> encoderReadings;
	private static int dataPoints = 35;
	
	public SimpleRegressionTest(String title) {
	    super(title);
	    
	    XYDataset dataset = createDataset();

	    JFreeChart chart = ChartFactory.createScatterPlot(
	        "Vuforia Distance vs Encoder Clicks Chart", 
	        "X-Axis", "Y-Axis", dataset);

	    
	    ChartPanel panel = new ChartPanel(chart);
	    setContentPane(panel);
	  }


	private XYDataset createDataset() {
		XYSeriesCollection dataset = new XYSeriesCollection();
		XYSeries series = new XYSeries("Actual");
		XYSeries prediction = new XYSeries("Regression");
		for (int i = 0; i < dataPoints; i++) {
			series.add(encoderReadings.get(i), vuforiaReadings.get(i));
			prediction.add(i * 300, getPrediction(i * 300));
		}
		dataset.addSeries(series);
		dataset.addSeries(prediction);
		return dataset;
	}
	
	private static void populateData() {
		vuforiaReadings = new ArrayList<>();
		encoderReadings = new ArrayList<>();

		vuforiaReadings.add(60.0);
		encoderReadings.add(0.0);

		for(int i = 1; i < dataPoints; i++) {
			vuforiaReadings.add(vuforiaReadings.get(i-1) - 5 * Math.random());
			encoderReadings.add(encoderReadings.get(i-1) + 500 * Math.random());
		}
	}
	
	private static void runRegression() {
		regression = new SimpleRegression();
		for(int i = 1; i < dataPoints; i++) {
			regression.addData(encoderReadings.get(i), vuforiaReadings.get(i));
		}
	}
	
	private static double getPrediction(int clicks) {
		return regression.getSlope() * clicks + regression.getIntercept();
	}
	
	private static void openChart() {
		SwingUtilities.invokeLater(() -> {
	      SimpleRegressionTest example = new SimpleRegressionTest("Regression Test");
	      example.setSize(800, 400);
	      example.setLocationRelativeTo(null);
	      example.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
	      example.setVisible(true);
	    });
	}

	public static void main(String[] args) {
		populateData();
		runRegression();
		openChart();
	}

}
