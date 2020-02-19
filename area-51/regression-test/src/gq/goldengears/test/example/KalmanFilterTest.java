package gq.goldengears.test.example;

import javax.swing.JFrame;
import javax.swing.SwingUtilities;
import javax.swing.WindowConstants;

import org.apache.commons.math3.filter.DefaultMeasurementModel;
import org.apache.commons.math3.filter.DefaultProcessModel;
import org.apache.commons.math3.filter.KalmanFilter;
import org.apache.commons.math3.filter.MeasurementModel;
import org.apache.commons.math3.filter.ProcessModel;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.random.JDKRandomGenerator;
import org.apache.commons.math3.random.RandomGenerator;
import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

public class KalmanFilterTest extends JFrame {

	public KalmanFilterTest(String title) {
	    super(title);
	    
	    XYDataset dataset = generateKalmanDataset();

	    JFreeChart chart = ChartFactory.createScatterPlot(
	        "Kalman Filter Test", 
	        "X-Axis", "Y-Axis", dataset);

	    
	    ChartPanel panel = new ChartPanel(chart);
	    setContentPane(panel);
	  }


	private XYDataset generateKalmanDataset() {
		
		double constantVoltage = 10d;
		double measurementNoise = 0.1d;
		double processNoise = 1e-5d;
		
//		A - state transition matrix
//		B - control input matrix
//		H - measurement matrix
//		Q - process noise covariance matrix
//		R - measurement noise covariance matrix
//		P - error covariance matrix

		// A = [ 1 ]
		RealMatrix A = new Array2DRowRealMatrix(new double[] { 1d });
		// B = null
		RealMatrix B = null;
		// H = [ 1 ]
		RealMatrix H = new Array2DRowRealMatrix(new double[] { 1d });
		// x = [ 10 ]
		RealVector x = new ArrayRealVector(new double[] { constantVoltage });
		// Q = [ 1e-5 ]
		RealMatrix Q = new Array2DRowRealMatrix(new double[] { processNoise });
		// P = [ 1 ]
		RealMatrix P0 = new Array2DRowRealMatrix(new double[] { 1d });
		// R = [ 0.1 ]
		RealMatrix R = new Array2DRowRealMatrix(new double[] { measurementNoise });

		ProcessModel pm = new DefaultProcessModel(A, B, Q, x, P0);
		MeasurementModel mm = new DefaultMeasurementModel(H, R);
		KalmanFilter filter = new KalmanFilter(pm, mm);  
		
		// for graphing
		XYSeriesCollection dataset = new XYSeriesCollection();
		XYSeries actualSeries = new XYSeries("Actual");
		XYSeries estimateSeries = new XYSeries("Estimate");

		// iterate 60 steps
		for (int i = 0; i < 60; i++) {
			// predict the state estimate one time-step ahead
		   // optionally provide some control input
		   filter.predict();

		   // obtain measurement vector z
		   RealVector z = getMeasurement();

		   // correct the state estimate with the latest measurement
		   filter.correct(z);
		   
		   // get estimated
		   double[] stateEstimate = filter.getStateEstimation();

		    // for graphing
			actualSeries.add(i, z.getEntry(0));
			estimateSeries.add(i, stateEstimate[0]);
		}
		// for graphing
		dataset.addSeries(actualSeries);
		dataset.addSeries(estimateSeries);
		
		return dataset;
	}
	
	private RealVector getMeasurement() {
		RandomGenerator random = new JDKRandomGenerator();
		RealVector noise = new ArrayRealVector(1);
		noise.setEntry(0, 0.5 * random.nextGaussian());
		return noise;


	}
	

	public static void main(String[] args) {
		
		     
		SwingUtilities.invokeLater(() -> {
	      KalmanFilterTest example = new KalmanFilterTest("Kalman FilterTest");
	      example.setSize(800, 400);
	      example.setLocationRelativeTo(null);
	      example.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
	      example.setVisible(true);
	    });

	
	}

}
