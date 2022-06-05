package com.swervedrivespecialties.swervelib.rev;

import com.revrobotics.*;
import com.swervedrivespecialties.swervelib.*;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

import static com.swervedrivespecialties.swervelib.rev.RevUtils.checkNeoError;

public final class NeoSteerControllerFactoryBuilder {
    // PID configuration
    private double pidProportional = Double.NaN;
    private double pidIntegral = Double.NaN;
    private double pidDerivative = Double.NaN;

    private double nominalVoltage = Double.NaN;
    private double currentLimit = Double.NaN;

    public NeoSteerControllerFactoryBuilder withPidConstants(double proportional, double integral, double derivative) {
        this.pidProportional = proportional;
        this.pidIntegral = integral;
        this.pidDerivative = derivative;
        return this;
    }

    public boolean hasPidConstants() {
        return Double.isFinite(pidProportional) && Double.isFinite(pidIntegral) && Double.isFinite(pidDerivative);
    }

    public NeoSteerControllerFactoryBuilder withVoltageCompensation(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public boolean hasVoltageCompensation() {
        return Double.isFinite(nominalVoltage);
    }

    public NeoSteerControllerFactoryBuilder withCurrentLimit(double currentLimit) {
        this.currentLimit = currentLimit;
        return this;
    }

    public boolean hasCurrentLimit() {
        return Double.isFinite(currentLimit);
    }

    public <T> SteerControllerFactory<ControllerImplementation, NeoSteerConfiguration<T>> build(AbsoluteEncoderFactory<T> encoderFactory) {
        return new FactoryImplementation<>(encoderFactory);
    }

    public class FactoryImplementation<T> implements SteerControllerFactory<ControllerImplementation, NeoSteerConfiguration<T>> {
        private final AbsoluteEncoderFactory<T> encoderFactory;

        public FactoryImplementation(AbsoluteEncoderFactory<T> encoderFactory) {
            this.encoderFactory = encoderFactory;
        }

        @Override
        public void addDashboardEntries(ShuffleboardContainer container, ControllerImplementation controller) {
            SteerControllerFactory.super.addDashboardEntries(container, controller);
            container.addNumber("Absolute Encoder Angle", () -> Math.toDegrees(controller.absoluteEncoder.getAbsoluteAngle()));
        }

        @Override
        public ControllerImplementation create(NeoSteerConfiguration<T> steerConfiguration, ModuleConfiguration moduleConfiguration) {
            AbsoluteEncoder absoluteEncoder = encoderFactory.create(steerConfiguration.getEncoderConfiguration());

            CANSparkMax motor = new CANSparkMax(steerConfiguration.getMotorPort(), CANSparkMaxLowLevel.MotorType.kBrushless);
            checkNeoError(motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100), "Failed to set periodic status frame 0 rate");
            checkNeoError(motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20), "Failed to set periodic status frame 1 rate");
            checkNeoError(motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20), "Failed to set periodic status frame 2 rate");
            checkNeoError(motor.setIdleMode(CANSparkMax.IdleMode.kBrake), "Failed to set NEO idle mode");
            motor.setInverted(!moduleConfiguration.isSteerInverted());
            if (hasVoltageCompensation()) {
                checkNeoError(motor.enableVoltageCompensation(nominalVoltage), "Failed to enable voltage compensation");
            }
            if (hasCurrentLimit()) {
                checkNeoError(motor.setSmartCurrentLimit((int) Math.round(currentLimit)), "Failed to set NEO current limits");
            }

            RelativeEncoder integratedEncoder = motor.getEncoder();
            checkNeoError(integratedEncoder.setPositionConversionFactor(2.0 * Math.PI * moduleConfiguration.getSteerReduction()), "Failed to set NEO encoder conversion factor");
            checkNeoError(integratedEncoder.setVelocityConversionFactor(2.0 * Math.PI * moduleConfiguration.getSteerReduction() / 60.0), "Failed to set NEO encoder conversion factor");
            checkNeoError(integratedEncoder.setPosition(absoluteEncoder.getAbsoluteAngle()), "Failed to set NEO encoder position");

            SparkMaxPIDController controller = motor.getPIDController();
            if (hasPidConstants()) {
                checkNeoError(controller.setP(pidProportional), "Failed to set NEO PID proportional constant");
                checkNeoError(controller.setI(pidIntegral), "Failed to set NEO PID integral constant");
                checkNeoError(controller.setD(pidDerivative), "Failed to set NEO PID derivative constant");
            }
            checkNeoError(controller.setFeedbackDevice(integratedEncoder), "Failed to set NEO PID feedback device");

            return new ControllerImplementation(motor, absoluteEncoder);
        }
    }

    public static class ControllerImplementation implements SteerController {
        private static final int ENCODER_RESET_ITERATIONS = 500;
        private static final double ENCODER_RESET_MAX_ANGULAR_VELOCITY = Math.toRadians(0.5);

        @SuppressWarnings({"FieldCanBeLocal", "unused"})
        private final CANSparkMax motor;
        private final SparkMaxPIDController controller;
        private final RelativeEncoder motorEncoder;
        private final AbsoluteEncoder absoluteEncoder;

        private double referenceAngleRadians = 0;

        private double resetIteration = 0;

        public ControllerImplementation(CANSparkMax motor, AbsoluteEncoder absoluteEncoder) {
            this.motor = motor;
            this.controller = motor.getPIDController();
            this.motorEncoder = motor.getEncoder();
            this.absoluteEncoder = absoluteEncoder;
        }

        @Override
        public double getReferenceAngle() {
            return referenceAngleRadians;
        }

        @Override
        public void setReferenceAngle(double referenceAngleRadians) {
            double currentAngleRadians = motorEncoder.getPosition();

            // Reset the NEO's encoder periodically when the module is not rotating.
            // Sometimes (~5% of the time) when we initialize, the absolute encoder isn't fully set up, and we don't
            // end up getting a good reading. If we reset periodically this won't matter anymore.
            if (motorEncoder.getVelocity() < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
                if (++resetIteration >= ENCODER_RESET_ITERATIONS) {
                    resetIteration = 0;
                    double absoluteAngle = absoluteEncoder.getAbsoluteAngle();
                    motorEncoder.setPosition(absoluteAngle);
                    currentAngleRadians = absoluteAngle;
                }
            } else {
                resetIteration = 0;
            }

            double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
            if (currentAngleRadiansMod < 0.0) {
                currentAngleRadiansMod += 2.0 * Math.PI;
            }

            // The reference angle has the range [0, 2pi) but the Neo's encoder can go above that
            double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
            if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
                adjustedReferenceAngleRadians -= 2.0 * Math.PI;
            } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
                adjustedReferenceAngleRadians += 2.0 * Math.PI;
            }

            this.referenceAngleRadians = referenceAngleRadians;

            controller.setReference(adjustedReferenceAngleRadians, CANSparkMax.ControlType.kPosition);
        }

        @Override
        public double getStateAngle() {
            double motorAngleRadians = motorEncoder.getPosition();
            motorAngleRadians %= 2.0 * Math.PI;
            if (motorAngleRadians < 0.0) {
                motorAngleRadians += 2.0 * Math.PI;
            }

            return motorAngleRadians;
        }
    }
}
