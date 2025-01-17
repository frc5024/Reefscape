package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class elevatorCmd extends Command{

    public final Elevator elevatorSubsystem;
    public boolean forward;

    public elevatorCmd(Elevator elevatorSubsystem, boolean forward) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.forward = forward;

        addRequirements(elevatorSubsystem);

    }


    @Override
    public void initialize() {
        if (forward) {
            elevatorSubsystem.activeMotor(true);
        }
        else {
            elevatorSubsystem.reverseMotor(true);
        }

    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.activeMotor(false);
    }

   
}
