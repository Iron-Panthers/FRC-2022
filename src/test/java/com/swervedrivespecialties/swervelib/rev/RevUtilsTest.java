package com.swervedrivespecialties.swervelib.rev;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertThrows;

import com.revrobotics.REVLibError;
import org.junit.jupiter.api.Test;

class RevUtilsTest {
  @Test
  void checkNeoError() {
    assertThrows(RuntimeException.class, () -> RevUtils.checkNeoError(REVLibError.kError, ""));
    assertThrows(
        RuntimeException.class, () -> RevUtils.checkNeoError(REVLibError.kCantFindFirmware, ""));
    assertDoesNotThrow(() -> RevUtils.checkNeoError(REVLibError.kOk, ""));
  }
}
