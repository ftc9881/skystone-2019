public class PIDTest {
	public static void main(String[] args) {
		PIDController test = new PIDController(0.05, 0.01, 0.05, 10);


		double pVar = -10;
		while (Math.abs(pVar-10)>0.1) {
			try {
				Thread.sleep(200);
			} catch (Exception ex) {
				System.out.println(ex);
			}

			pVar += test.getCorrectedOutput(pVar);
			System.out.println(pVar);
			
		}
	}
}


class PIDController {
    private double currentTime, previousTime, deltaTime;
    private double integral = 0;
    private double currentError, previousError, deltaError, derivative;
    private double kP, kI, kD, targetValue;

    public PIDController (double kP, double kI, double kD, double targetValue) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.targetValue = targetValue;
        this.currentTime = System.currentTimeMillis()/1000.0;
    }

    public double getCorrectedOutput(double processValue) {
        previousTime = currentTime;
        
        currentTime = System.currentTimeMillis()/1000.0;
        
        deltaTime = currentTime - previousTime;

        currentError = processValue - targetValue;
        deltaError = currentError - previousError;
        derivative =  deltaError / deltaTime;

        integral += currentError * deltaTime;

        previousError = currentError;
        double output = -(currentError * kP + integral * kI + derivative * kD);
        return output;
    }

}

