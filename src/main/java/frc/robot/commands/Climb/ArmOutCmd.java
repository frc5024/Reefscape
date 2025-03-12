package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class ArmOutCmd extends Command {

    private Climb climbSubsystem;

    public ArmOutCmd(Climb climbSubsystem) {
        this.climbSubsystem = climbSubsystem;

        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
    }

    public void execute() {
        climbSubsystem.extendArm();
    }

    @Override
    public void end(boolean interrupted) {
        climbSubsystem.stopMotor();
        climbSubsystem.extended = true;
    }

    @Override
    public boolean isFinished() {
        // Checks if climb arm is at extended position
        return climbSubsystem.isExtended();
    }
}