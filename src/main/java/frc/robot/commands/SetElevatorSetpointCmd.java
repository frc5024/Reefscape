package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class SetElevatorSetpointCmd extends Command{

    public final Elevator elevatorSubsystem;
    public boolean forward;
    public double setpoint;

    public SetElevatorSetpointCmd(Elevator elevatorSubsystem, double setpoint) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.setpoint = setpoint;

        addRequirements(elevatorSubsystem);

    }


    @Override
    public void initialize() {
        elevatorSubsystem.setSetPoint(setpoint);
        elevatorSubsystem.motorOn(true);
        System.out.println("aaAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");

    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.motorOn(false);
    }

   
}
