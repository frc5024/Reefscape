package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.elevatorConstants;

public class ElevatorPID extends PIDSubsystem{
    
    public ElevatorPID(){
        super (new PIDController(elevatorConstants.kP,elevatorConstants.kI,elevatorConstants.kD));
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'useOutput'");
    }

    @Override
    protected double getMeasurement() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getMeasurement'");
    }
}
