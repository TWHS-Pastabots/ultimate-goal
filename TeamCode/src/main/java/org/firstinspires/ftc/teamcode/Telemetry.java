package org.firstinspires.ftc.teamcode;

/**
 * A class containing utility functions for printing telemetry data to the Driver Station log output.
 *
 * @author Ryan Walters
 */
public class Telemetry {
    static final int barWidth = 34; // The width of the telemetry terminal
    static final String space = "\u2591"; // Light shade block
    static final String bar = "\u2588"; // Full block

    /**
     * Create a progressbar out of Unicode characters.
     * Unicode characters for progress bar found <a href="https://charbase.com/block/block-elements">here</a> and <a href="http://jkorpela.fi/chars/spaces.html">here</a>.
     *
     * @param level The level of the progressbar to be rendered.
     * @return A progressbar made out of Unicode block and space characters with a certain percent filling.
     */
    public static String createLevel(float level) {
        StringBuilder builder = new StringBuilder("[");
        int halfWidth = barWidth / 2;
        int barCount = Math.round(Math.abs(level) * halfWidth);

        // Credit to Matt P. (@BSFishy) for the idea - expanding the bar to show signage like a numberline
        builder.append(Util.repeat(space, halfWidth));
        builder.append(Util.repeat(bar, barCount));
        builder.append(Util.repeat(space, halfWidth - barCount));
        builder.append("]");

        if (level < 0)
            builder.reverse();

        return builder.toString();
    }
}
