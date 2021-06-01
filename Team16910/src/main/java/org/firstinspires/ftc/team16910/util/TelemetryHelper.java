package org.firstinspires.ftc.team16910.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Assert;

/**
 * TODO(BSFishy): document this
 *
 * @author Matt Provost &lt;mattprovost6@gmail.com&gt;
 */
public class TelemetryHelper implements Telemetry {
    private final Telemetry telemetry;
    private final FtcDashboard dashboard;
    private TelemetryPacket packet;

    /**
     * TODO(BSFishy): document this
     *
     * @param telemetry the previous telemetry to use
     * @param dashboard the dashboard
     */
    public TelemetryHelper(Telemetry telemetry, FtcDashboard dashboard) {
        Assert.assertNotNull(telemetry, "Telemetry must not be null");
        Assert.assertNotNull(dashboard, "The dashboard must not be null");

        this.telemetry = telemetry;
        this.dashboard = dashboard;
        this.packet = null;
    }

    /**
     * TODO(BSFishy): document this
     *
     * @return the current dashboard packet
     */
    public TelemetryPacket getPacket() {
        return packet;
    }

    /**
     * TODO(BSFishy): document this
     *
     * @param packet the new dashboard packet
     */
    public void setPacket(TelemetryPacket packet) {
        this.packet = packet;
    }

    /**
     * TODO(BSFishy): document this
     */
    public void newPacket() {
        setPacket(new TelemetryPacket());
    }

    @Override
    public Item addData(String caption, String format, Object... args) {
        if (packet != null) {
            packet.put(caption, String.format(format, args));
        }

        return telemetry.addData(caption, format, args);
    }

    @Override
    public Item addData(String caption, Object value) {
        if (packet != null) {
            packet.put(caption, value);
        }

        return telemetry.addData(caption, value);
    }

    @Override
    public <T> Item addData(String caption, Func<T> valueProducer) {
        return telemetry.addData(caption, valueProducer);
    }

    @Override
    public <T> Item addData(String caption, String format, Func<T> valueProducer) {
        return telemetry.addData(caption, format, valueProducer);
    }

    @Override
    public boolean removeItem(Item item) {
        return telemetry.removeItem(item);
    }

    @Override
    public void clear() {
        telemetry.clear();
    }

    @Override
    public void clearAll() {
        telemetry.clearAll();
    }

    @Override
    public Object addAction(Runnable action) {
        return telemetry.addAction(action);
    }

    @Override
    public boolean removeAction(Object token) {
        return telemetry.removeAction(token);
    }

    @Override
    public void speak(String text) {
        telemetry.speak(text);
    }

    @Override
    public void speak(String text, String languageCode, String countryCode) {
        telemetry.speak(text, languageCode, countryCode);
    }

    @Override
    public boolean update() {
        if (packet != null) {
            dashboard.sendTelemetryPacket(packet);
        }

        return telemetry.update();
    }

    @Override
    public Line addLine() {
        return telemetry.addLine();
    }

    @Override
    public Line addLine(String lineCaption) {
        if (packet != null) {
            packet.addLine(lineCaption);
        }

        return telemetry.addLine(lineCaption);
    }

    @Override
    public boolean removeLine(Line line) {
        return telemetry.removeLine(line);
    }

    @Override
    public boolean isAutoClear() {
        return telemetry.isAutoClear();
    }

    @Override
    public void setAutoClear(boolean autoClear) {
        telemetry.setAutoClear(autoClear);
    }

    @Override
    public int getMsTransmissionInterval() {
        return telemetry.getMsTransmissionInterval();
    }

    @Override
    public void setMsTransmissionInterval(int msTransmissionInterval) {
        telemetry.setMsTransmissionInterval(msTransmissionInterval);
        dashboard.setTelemetryTransmissionInterval(msTransmissionInterval);
    }

    @Override
    public String getItemSeparator() {
        return telemetry.getItemSeparator();
    }

    @Override
    public void setItemSeparator(String itemSeparator) {
        telemetry.setItemSeparator(itemSeparator);
    }

    @Override
    public String getCaptionValueSeparator() {
        return telemetry.getCaptionValueSeparator();
    }

    @Override
    public void setCaptionValueSeparator(String captionValueSeparator) {
        telemetry.setCaptionValueSeparator(captionValueSeparator);
    }

    @Override
    public void setDisplayFormat(DisplayFormat displayFormat) {
        telemetry.setDisplayFormat(displayFormat);
    }

    @Override
    public Log log() {
        return telemetry.log();
    }
}
