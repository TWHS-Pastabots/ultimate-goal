package org.firstinspires.ftc.team15021.gamepad;

import java.util.ArrayList;
import java.util.List;



class InstallableBoolean {
    private List<BooleanStrategy> installed = new ArrayList<>();
    private boolean initial = false;

    public InstallableBoolean() {
    }

    public void installStrategy(BooleanStrategy booleanStrategy) {
        installed.add(booleanStrategy);
    }

    public void update(boolean next) {
        if (installed.size() > 0) {
            for (BooleanStrategy booleanStrategy : installed)
                booleanStrategy.update(next);
        }
    }
}


class InstallableFloat {
    private List<FloatStrategy> installed = new ArrayList<>();
    private float initial = 0f;

    public InstallableFloat() {
    }

    public void installStrategy(FloatStrategy floatStrategy) {
        installed.add(floatStrategy);
    }

    /**
     * Passes on updated floating point values to all installed strategies.
     *
     * @param next The latest floating point value.
     */
    public void update(float next) {
        if (this.isUpdatable()) {
            for (FloatStrategy floatStrategy : installed)
                floatStrategy.update(next);
        }
    }

    /**
     * @return Returns true if this Installable instance has any strategy instances to update.
     */
    public boolean isUpdatable() {
        return installed.size() > 0;
    }
}