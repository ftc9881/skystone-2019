package teamcode;

public class Main {
	
	public static void main(String[] args) {
		ActionThread action = new ExampleAction(100);
		IEndCondition condition = new Timeout(2000);
		runTask(action, condition);

		System.out.println("outside task");
	}
	
	 private static void runTask(ActionThread action, IEndCondition endCondition) {
		action.start();
		endCondition.start();

		// If end condition completes before action, then action is stopped.
		// If action completes before end condition, then complete.
		while (!action.isStopped() && !endCondition.isTrue()) {
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}

		action.stop();
		endCondition.stop();
		
		System.out.println("task completed");

	}

}
