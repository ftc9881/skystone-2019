package org.firstinspires.ftc.teamcode.auto.structure;

import java.util.ArrayList;

public class CombinedConditions implements IEndCondition {
    private ArrayList<IEndCondition> conditions;

    public CombinedConditions() {
        this.conditions = new ArrayList<IEndCondition>();
    }

    public CombinedConditions(ArrayList<IEndCondition> conditions) {
        this.conditions = conditions;
    }

    public CombinedConditions add(IEndCondition condition) {
        conditions.add(condition);
        return this;
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
