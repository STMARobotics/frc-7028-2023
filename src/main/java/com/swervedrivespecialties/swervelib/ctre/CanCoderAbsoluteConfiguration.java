package com.swervedrivespecialties.swervelib.ctre;

public class CanCoderAbsoluteConfiguration {
    private final int id;
    private final double offset;

    public CanCoderAbsoluteConfiguration(int id, double offset) {
        this.id = id;
        this.offset = offset;
    }

    public int getId() {
        return id;
    }

    public double getOffset() {
        return offset;
    }
}
