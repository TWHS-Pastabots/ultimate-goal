package org.firstinspires.ftc.teamcode;

public class Util {
    // https://charbase.com/block/block-elements
    // http://jkorpela.fi/chars/spaces.html

    static final int barWidth = 34;
    static final String space = "\u2591"; // Light shade block
    static final String bar = "\u2588"; // Full block

    /**
     * Like Math.max, but dependent on magnitude (ignores signage)
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
     * @param duration The duration of the time to be printed
     * @return A human readable representation of a positive duration
     */
    public static String getHumanDuration(float duration) {
        if (duration > 3600) {
            int hours = Math.round(duration / 3600);
            int minutes = Math.round((duration % 3600) / 60);
            int seconds = Math.round(duration % 60);
            return String.format("%d hours, %d minutes and %d seconds", hours, minutes, seconds);
        } else if (duration > 60) {
            int minutes = Math.round(duration / 60);
            int seconds = Math.round(duration % 60);
            return String.format("%d minutes and %d seconds", minutes, seconds);
        } else if (duration > 0) {
            return String.format("%d seconds", Math.round(duration));
        }
        return "";
    }

    /**
     * Create a progressbar out of Unicode characters.
     *
     * @param level The level of the progressbar to be rendered.
     * @return A progressbar made out of Unicode block and space characters with a certain percent filling.
     */
    public static String createLevel(float level) {
        StringBuilder builder = new StringBuilder("[");
        int halfWidth = Util.barWidth / 2;
        int barCount = Math.round(Math.abs(level) * halfWidth);

        // Credit to Matt P. (@BSFishy) for the idea - expanding the bar to show signage like a numberline
        builder.append(repeat(Util.space, halfWidth));
        builder.append(repeat(Util.bar, barCount));
        builder.append(repeat(Util.space, halfWidth - barCount));
        builder.append("]");

        if (level < 0)
            builder.reverse();

        return builder.toString();
    }

    /**
     * Easing function from https://easings.net/#easeInSine
     *
     * @param number A value in the 0.0 to 1.0 range
     * @return A value with the easeInSine easing function applied
     */
    public static float easeInSine(float number) {
        return (float) (1 - Math.cos((number * Math.PI) / 2));
    }
}
