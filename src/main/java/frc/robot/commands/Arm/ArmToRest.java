package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmToRest extends CommandBase {
    ArmSubsystem arm;
    
    /**
     * Creates a new ArmCommand
     */
    public ArmToRest(ArmSubsystem arm) {
        this.arm = arm;
    }

    // Called when command is initially scheduled
    @Override
    public void initialize() {
    }

    // Called when scheduler runs when command is initially scheduled
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted
    @Override
    public void end(boolean interrupted) {

    }

    // 
    @Override
    public boolean isFinished() {
        return false;
    }
}
