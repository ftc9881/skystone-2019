package teamcode;

public class ExampleAction extends ActionThread {
	
	double startTime;
	double waitTime;

    public ExampleAction(double milliseconds) {
        this.startTime = System.currentTimeMillis();
        this.waitTime = milliseconds;
    }

	@Override
	protected void onRun() {
		System.out.println("Action: On Run");
	}

	@Override
	protected boolean runIsComplete() {
    	boolean timerDone = (System.currentTimeMillis() - startTime) >= waitTime;
    	System.out.println("Action: is done? " + timerDone);
        return timerDone;
	}

	@Override
	public void insideRun() {
		System.out.println("Action: Inside Run");
	}

	@Override
	protected void onEndRun() {
		System.out.println("Action: End Run");
	}

}
