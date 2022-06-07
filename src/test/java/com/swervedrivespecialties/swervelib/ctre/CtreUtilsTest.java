package com.swervedrivespecialties.swervelib.ctre;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertThrows;

import com.ctre.phoenix.ErrorCode;

import org.junit.jupiter.api.Test;

import frc.robot.swerve.CtreUtils;

class CtreUtilsTest {
    @Test
    void checkNeoError() {
        assertThrows(RuntimeException.class, () -> CtreUtils.checkCtreError(ErrorCode.GeneralError, ""));
        assertThrows(RuntimeException.class, () -> CtreUtils.checkCtreError(ErrorCode.FirmVersionCouldNotBeRetrieved, ""));
        assertDoesNotThrow(() -> CtreUtils.checkCtreError(ErrorCode.OK, ""));
    }
}
