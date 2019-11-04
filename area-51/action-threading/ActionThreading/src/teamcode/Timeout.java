package teamcode;

public class Timeout implements IEndCondition {
    double startTime;
    double waitTime;

    public Timeout(double milliseconds) {
        this.startTime = System.currentTimeMillis();
        this.waitTime = milliseconds;
    }

    @Override
    public boolean isTrue() {
    	boolean timerDone = (System.currentTimeMillis() - startTime) >= waitTime;
    	System.out.println("Condition: is done? " + timerDone);
        return timerDone;
    }
    
    @Override
    public void start() {
    	System.out.println("Condition: start");
    }
    
    public void stop() {
    	System.out.println("Condition: stop");
    }
}
