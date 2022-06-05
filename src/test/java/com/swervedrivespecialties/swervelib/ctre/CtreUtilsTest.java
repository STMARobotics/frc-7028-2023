package com.swervedrivespecialties.swervelib.ctre;

import com.ctre.phoenix.ErrorCode;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertThrows;

class CtreUtilsTest {
    @Test
    void checkNeoError() {
        assertThrows(RuntimeException.class, () -> CtreUtils.checkCtreError(ErrorCode.GeneralError, ""));
        assertThrows(RuntimeException.class, () -> CtreUtils.checkCtreError(ErrorCode.FirmVersionCouldNotBeRetrieved, ""));
        assertDoesNotThrow(() -> CtreUtils.checkCtreError(ErrorCode.OK, ""));
    }
}
