package org.firstinspires.ftc.teamcode.math;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.opencv.core.Point;
import org.opencv.core.Rect;

import java.util.List;

public class GeneralMath {


    public enum Conditional {
        GREATER, LESS, CLOSE, NONE;

        static public Conditional convertString(String string) {
            switch (string.toUpperCase()) {
                case "GREATER":
                    return GREATER;
                case "LESS":
                    return LESS;
                case "NONE":
                    return NONE;
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
                case NONE:
                    return true;
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
        return standardDeviation(numbers, mean);
    }

    public static double standardDeviation(List<Number> numbers, double mean) {
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
        return Range.clip(Math.abs(power), min, 1.0) * (power < 0 ? -1 : 1);
    }

    public static double distance(Point a, Point b) {
        return Math.sqrt(Math.pow(b.x - a.x, 2) + Math.pow(b.y - a.y, 2));
    }

    public static Point getCenterPoint(Rect rect) {
        return new Point(rect.x + rect.width/2, rect.y + rect.height/2);
    }

    public static boolean isOutlier(List<Number> numbers, double value, double maxDeviation) {
        double mean = mean(numbers);
        return maxDeviation < Math.abs(value - mean) / standardDeviation(numbers, mean);
    }

    public static boolean isWithin(double a, double range, double b) {
        return a >= (b - range) && a <= (b + range);
    }

    public static int roundNearestMultiple(double value, int multiple) {
        return (int) (multiple * (Math.round(value/multiple)));
    }

}
