package org.firstinspires.ftc.team15021;

import java.util.Locale;

/**
 * A utility class storing miscellaneous functions such as for converting time, easing values,
 * String manipulation and other minor but repeated operations that would benefit from encapsulation.
 *
 * @author Ryan Walters
 */
public class Util {
    /**
     * Similar to <code>Math.max</code>, but dependent on magnitude (ignores signage).
     *
     * @return The float with the highest magnitude, preferring param A.
     */
    public static float maxMagnitude(float a, float b) {
        return Math.abs(a) >= Math.abs(b) ? a : b;
    }

    /**
     * @param sub    The String to be repeated
     * @param repeat The number of times to repeat the String
     * @return A new string made out of the sub string repeated a certain number of times.
     */
    public static String repeat(String sub, int repeat) {
        return new String(new char[repeat]).replace("\0", sub);
    }

    /**
     * A simple function for getting a human readable representation of a time duration.
     * Durations of 24 hours are not properly supported.
     *
     * @param duration The duration of the time to be printed
     * @return A human readable representation of a positive duration
     */
    public static String getHumanDuration(float duration) {
        if (duration > 3600) {
            int hours = Math.round(duration / 3600);
            int minutes = Math.round((duration % 3600) / 60);
            int seconds = Math.round(duration % 60);
            return String.format(Locale.ENGLISH, "%d hours, %d minutes and %d seconds", hours, minutes, seconds);
        } else if (duration > 60) {
            int minutes = Math.round(duration / 60);
            int seconds = Math.round(duration % 60);
            return String.format(Locale.ENGLISH, "%d minutes and %d seconds", minutes, seconds);
        } else if (duration >= 1) {
            return String.format(Locale.ENGLISH, "%d seconds", Math.round(duration));
        } else if (duration < 0) {
            return "Invalid duration";
        }
        return "Less than a second";
    }

    /**
     * A cubic easing function that keeps the sign of the function.
     *
     * @param number A value in the [<code>0.0</code>, <code>1.0</code>] range.
     * @return A value with the cubic easing function applied
     */
    public static double cubicEasing(double number) {
        return number * number * number;
    }

    /**
     * A square easing function that keeps the sign of the function.
     *
     * @param number A value in the [<code>0.0</code>, <code>1.0</code>] range.
     * @return A value with the square easing function applied.
     */
    public static double squareEasing(double number) {
        if (number > 0) return number * number;
        else if (number < 0) return -(number * number);
        return 0.0;
    }

    /**
     * Helper function for wrapping index values, especially when incrementing and decrementing.
     *
     * @param i      An integer index of any sign or magnitude.
     * @param length The length of the array you want to wrap indexes around.
     * @return The expected index, accounting for negative and positive wrapping.
     */
    public static int wrap(int i, int length) {
        if (i < 0)
            return length - Math.abs(i) % length;
        return i % length;
    }


    /**
     * @param value The value to be clamped
     * @param min   The maximum in the clamp range
     * @param max   The minimum in the clamp range
     * @return A value within the min/max clamp range - equal to the given value or the closest value within the range.
     */
    public static float clamp(float value, float min, float max) {
        return Math.max(min, Math.min(max, value));
    }
}
