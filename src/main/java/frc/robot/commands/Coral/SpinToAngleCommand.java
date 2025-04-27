package frc.robot.commands.Coral;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.coralConstants;
import frc.robot.subsystems.Coral;

public class SpinToAngleCommand extends Command {

    private final Coral coralSubsystem;

    private PIDController PID;
    double angleOutput;
    
    //constructor for IntakeCommand
    public SpinToAngleCommand(Coral coralSubsystem) {
        this.coralSubsystem = coralSubsystem;

        PID = new PIDController(coralConstants.kP, coralConstants.kI, coralConstants.kD);

        addRequirements(coralSubsystem);
    }

    public void initialize() {
        PID.reset();
    }

    public void execute() {
        double currentAngle = coralSubsystem.getEncoder();
        double targetAngle = coralConstants.targetAngle; 
        
        double goToAngle = coralConstants.targetAngle - currentAngle;

        if(Math.abs(coralSubsystem.getEncoder() - targetAngle) > 1) {
            angleOutput = PID.calculate(goToAngle,0); //or (currentAngle, targetAngle)
            coralSubsystem.coralMotor.set(angleOutput);

        } else {
            coralSubsystem.coralMotor.set(0);
        }
        }

    public void end(boolean interrupted) {
        coralSubsystem.coralMotor.set(0); // Stop the motor
    }

    @Override
    public boolean isFinished() {
        return Math.abs(coralSubsystem.getEncoder() - coralConstants.targetAngle) <= 1; // Within tolerance
    }
        
    }
