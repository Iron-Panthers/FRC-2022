package com.swervedrivespecialties.swervelib.rev;

import com.revrobotics.RevLibError;

public final class RevUtils {
    private RevUtils() {
    }

    public static void checkNeoError(RevLibError error, String message) {
        if (error != RevLibError.kOk) {
            throw new RuntimeException(String.format("%s: %s", message, error.toString()));
        }
    }
}
