package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends CommandBase {
    ArmSubsystem arm;
    double target;
    
    /**
     * Creates a new ArmCommand
     */
    public ArmCommand(ArmSubsystem arm, double angle) {
        this.arm = arm;
        this.target = angle;
    }

    // Called when command is initially scheduled
    @Override
    public void initialize() {
        arm.stopMotor();
    }

    // Called when scheduler runs when command is initially scheduled
    @Override
    public void execute() {
        arm.setPower(1);
    }

    // Called once the command ends or is interrupted
    @Override
    public void end(boolean interrupted) {
        arm.stopMotor();
    }

    // 
    @Override
    public boolean isFinished() {
        return false;
    }
}
