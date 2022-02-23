package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends CommandBase {
    ArmSubsystem arm;
    
    /**
     * Creates a new ArmCommand
     */
    public ArmCommand(ArmSubsystem arm) {
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
