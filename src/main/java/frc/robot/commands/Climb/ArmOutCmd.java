package frc.robot.commands.Climb;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;

public class ArmOutCmd extends Command {

    private Climb climbSubsystem;

    private PIDController armPID;
    double armPIDoutput;

    public ArmOutCmd(Climb climbSubsystem) {
        this.climbSubsystem = climbSubsystem;

        armPID = new PIDController(0.07, 0, 0.005);

        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
        armPID.reset();
    }

    public void execute() {
        double distance = climbSubsystem.getEncoder() - Constants.ClimbConstants.extendedPosition;

        if (Math.abs(climbSubsystem.getEncoder() - Constants.ClimbConstants.extendedPosition) > 1) {
            armPIDoutput = armPID.calculate(distance, 0);
            climbSubsystem.moveMotor(armPIDoutput);
        } else {
            climbSubsystem.stopMotor();
        }
    }

    @Override
    public void end(boolean interrupted) {
        climbSubsystem.stopMotor();
        climbSubsystem.extended = true;
    }

    @Override
    public boolean isFinished() {
        // Checks if climb arm is at extended position
        return false;
    }
}