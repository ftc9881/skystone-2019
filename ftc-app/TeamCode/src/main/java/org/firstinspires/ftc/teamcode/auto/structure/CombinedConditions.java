package org.firstinspires.ftc.teamcode.auto.structure;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class CombinedConditions implements IEndCondition {
    private List<IEndCondition> conditions;

    public CombinedConditions(IEndCondition ... conditions) {
        this.conditions = convertToList(conditions);
    }

    public void add(IEndCondition ... conditions) {
        this.conditions.addAll(convertToList(conditions));
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

    private List<IEndCondition> convertToList(IEndCondition ... conditions) {
        List<IEndCondition> list = new ArrayList<>();
        Collections.addAll(list, conditions);
        return list;
    }
}
