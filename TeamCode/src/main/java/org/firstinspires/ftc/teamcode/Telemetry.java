package org.firstinspires.ftc.teamcode;

/**
 * A class containing utility functions for printing telemetry data to the Driver Station log output.
 *
 * @author Ryan Walters
 */
public class Telemetry {
    /**
     * Create a progressbar out of Unicode characters.
     * Unicode characters for progress bar found <a href="https://charbase.com/block/block-elements">here</a> and <a href="http://jkorpela.fi/chars/spaces.html">here</a>.
     *
     * @param level The level of the progressbar to be rendered.
     * @return A progressbar made out of Unicode block and space characters with a certain percent filling.
     */
    public static String createLevel(float level) {
        StringBuilder builder = new StringBuilder("[");
        int halfWidth = Util.barWidth / 2;
        int barCount = Math.round(Math.abs(level) * halfWidth);

        // Credit to Matt P. (@BSFishy) for the idea - expanding the bar to show signage like a numberline
        builder.append(Util.repeat(Util.space, halfWidth));
        builder.append(Util.repeat(Util.bar, barCount));
        builder.append(Util.repeat(Util.space, halfWidth - barCount));
        builder.append("]");

        if (level < 0)
            builder.reverse();

        return builder.toString();
    }
}
