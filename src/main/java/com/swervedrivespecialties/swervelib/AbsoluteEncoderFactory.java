package com.swervedrivespecialties.swervelib;

@FunctionalInterface
public interface AbsoluteEncoderFactory<Configuration> {
    AbsoluteEncoder create(Configuration configuration);
}
