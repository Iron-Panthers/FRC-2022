package com.swervedrivespecialties.swervelib.rev;

import com.revrobotics.RevLibError;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertThrows;

class RevUtilsTest {
    @Test
    void checkNeoError() {
        assertThrows(RuntimeException.class, () -> RevUtils.checkNeoError(RevLibError.kError, ""));
        assertThrows(RuntimeException.class, () -> RevUtils.checkNeoError(RevLibError.kCantFindFirmware, ""));
        assertDoesNotThrow(() -> RevUtils.checkNeoError(RevLibError.kOk, ""));
    }
}
