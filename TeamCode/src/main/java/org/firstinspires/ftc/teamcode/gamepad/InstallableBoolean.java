package org.firstinspires.ftc.teamcode.gamepad;

import java.util.ArrayList;
import java.util.List;

public class InstallableBoolean {
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