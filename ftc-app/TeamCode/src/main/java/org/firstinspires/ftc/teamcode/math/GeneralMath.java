package org.firstinspires.ftc.teamcode.math;

import java.util.List;

public class GeneralMath {
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
