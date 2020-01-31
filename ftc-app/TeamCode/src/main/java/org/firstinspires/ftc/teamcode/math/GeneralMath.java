package org.firstinspires.ftc.teamcode.math;

import com.qualcomm.robotcore.util.Range;

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

    public static double mean(List<Number> numbers) {
        double total = 0;
        for (Number number : numbers) {
            total += number.doubleValue();
        }
        return total / numbers.size();
    }

    public static double standardDeviation(List<Number> numbers) {
        double mean = mean(numbers);
        double stdDev = 0;
        for (Number number : numbers) {
            stdDev += Math.pow(number.doubleValue() - mean, 2);
        }
        return Math.sqrt(stdDev);
    }

    public static double round(double number, int places) {
        double placeholders = Math.pow(10, places);
        return Math.floor(number * placeholders) / placeholders;
    }

    public static double clipPower(double power) {
        return clipPower(power, 0);
    }

    public static double clipPower(double power, double min) {
        return Range.clip(Math.abs(power), min, 1.0) * (power < 1 ? -1 : 1);
    }

}
