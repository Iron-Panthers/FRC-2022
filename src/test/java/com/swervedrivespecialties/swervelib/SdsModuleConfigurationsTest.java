package com.swervedrivespecialties.swervelib;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class SdsModuleConfigurationsTest {
  @Test
  void mk3StandardReductions() {
    assertEquals(1.0 / 8.16, SdsModuleConfigurations.MK3_STANDARD.getDriveReduction(), 1e-4);
    assertEquals(1.0 / 12.8, SdsModuleConfigurations.MK3_STANDARD.getSteerReduction(), 1e-4);
  }

  @Test
  void mk3FastReductions() {
    assertEquals(1.0 / 6.86, SdsModuleConfigurations.MK3_FAST.getDriveReduction(), 1e-4);
    assertEquals(1.0 / 12.8, SdsModuleConfigurations.MK3_FAST.getSteerReduction(), 1e-4);
  }
}
