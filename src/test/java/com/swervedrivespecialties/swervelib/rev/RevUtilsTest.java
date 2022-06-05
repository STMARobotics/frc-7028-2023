package com.swervedrivespecialties.swervelib.rev;

import com.revrobotics.REVLibError;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertThrows;

class RevUtilsTest {
    @Test
    void checkNeoError() {
        assertThrows(RuntimeException.class, () -> RevUtils.checkNeoError(REVLibError.kError, ""));
        assertThrows(RuntimeException.class, () -> RevUtils.checkNeoError(REVLibError.kCantFindFirmware, ""));
        assertDoesNotThrow(() -> RevUtils.checkNeoError(REVLibError.kOk, ""));
    }
}
