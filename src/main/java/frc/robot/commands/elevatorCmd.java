package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class elevatorCmd extends Command{

    public final Elevator elevatorSubsystem;

    public elevatorCmd(Elevator elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;

        addRequirements(elevatorSubsystem);

    }


    @Override
    public void initialize() {
        elevatorSubsystem.activeMotor(true);

    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.activeMotor(false);
    }
}
