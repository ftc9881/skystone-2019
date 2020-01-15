package org.firstinspires.ftc.teamcode.auto.structure;

public class CombinedConditions implements IEndCondition {
    private IEndCondition[] conditions;

    public CombinedConditions(IEndCondition ... conditions) {
        this.conditions = conditions;
    }

    @Override
    public boolean isTrue() {
        boolean oneIsTrue = false;
        for (IEndCondition condition: conditions) {
            if (condition.isTrue()) {
                oneIsTrue = true;
                break;
            }
        }
        return oneIsTrue;
    }

    @Override
    public void start() {
        for (IEndCondition condition: conditions) {
            condition.start();
        }
    }

    @Override
    public void stop() {
        for (IEndCondition condition: conditions) {
            condition.stop();
        }
    }
}
