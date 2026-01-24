package org.jmhsrobotics.frc2026.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        
    }

    public default void updateInputs(IntakeIOInputs inputs) {}
}
