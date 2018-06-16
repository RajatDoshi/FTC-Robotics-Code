package net.kno3.util;

import com.google.common.base.Supplier;

/**
 * Created by robotics on 12/31/2017.
 */

public class TimeStopParameter implements Supplier<Boolean> {
    private long stopTime;

    public TimeStopParameter(double seconds) {
        this.stopTime = System.currentTimeMillis() + (long) (seconds * 1000);
    }

    @Override
    public Boolean get() {
        return System.currentTimeMillis() >= stopTime;
    }
}
