package com.swervedrivespecialties.swervelib;

import java.util.Objects;

/**
 * Additional Mk3 module configuration parameters.
 * <p>
 * The configuration parameters here are used to customize the behavior of the Mk3 swerve module.
 * Each setting is initialized to a default that should be adequate for most use cases.
 */
public class Mk3ModuleConfiguration {
    private double nominalVoltage = 12.0;
    private double driveCurrentLimit = 80.0;
    private double steerCurrentLimit = 20.0;

    public double getNominalVoltage() {
        return nominalVoltage;
    }

    public void setNominalVoltage(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
    }

    public double getDriveCurrentLimit() {
        return driveCurrentLimit;
    }

    public void setDriveCurrentLimit(double driveCurrentLimit) {
        this.driveCurrentLimit = driveCurrentLimit;
    }

    public double getSteerCurrentLimit() {
        return steerCurrentLimit;
    }

    public void setSteerCurrentLimit(double steerCurrentLimit) {
        this.steerCurrentLimit = steerCurrentLimit;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Mk3ModuleConfiguration that = (Mk3ModuleConfiguration) o;
        return Double.compare(that.getNominalVoltage(), getNominalVoltage()) == 0 && Double.compare(that.getDriveCurrentLimit(), getDriveCurrentLimit()) == 0 && Double.compare(that.getSteerCurrentLimit(), getSteerCurrentLimit()) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(getNominalVoltage(), getDriveCurrentLimit(), getSteerCurrentLimit());
    }

    @Override
    public String toString() {
        return "Mk3ModuleConfiguration{" +
                "nominalVoltage=" + nominalVoltage +
                ", driveCurrentLimit=" + driveCurrentLimit +
                ", steerCurrentLimit=" + steerCurrentLimit +
                '}';
    }
}
