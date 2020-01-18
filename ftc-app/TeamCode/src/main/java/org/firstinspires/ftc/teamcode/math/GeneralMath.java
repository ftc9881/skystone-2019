package org.firstinspires.ftc.teamcode.math;

import java.util.List;

public class GeneralMath {

    public enum Conditional {
        GREATER, LESS, CLOSE;

        static public Conditional convertString(String string) {
            switch (string.toUpperCase()) {
                case "GREATER":
                    return GREATER;
                case "LESS":
                    return LESS;
                default:
                    return CLOSE;
            }
        }

        public boolean evaluate(double actual, double target, double closeThreshold) {
            switch (this) {
                case GREATER:
                    return actual >= target;
                case LESS:
                    return actual <= target;
                case CLOSE:
                    return Math.abs(actual - target) < closeThreshold;
            }
            return false;
        }
    }

    public static double mean(List<Integer> numbers) {
        double total = 0;
        for (Integer number : numbers) {
            total += number;
        }
        return total / numbers.size();
    }

    public static double standardDeviation(List<Integer> numbers) {
        double mean = mean(numbers);
        double stdDev = 0;
        for (Integer number : numbers) {
            stdDev += Math.pow(number - mean, 2);
        }
        return Math.sqrt(stdDev);
    }

}
