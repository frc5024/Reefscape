package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.elevatorConstants;


public class Elevator extends SubsystemBase{
    private TalonFX elevatorMotor;
    

    public Elevator() {
        elevatorMotor = new TalonFX(0);

        super (new PIDController(elevatorConstants.kP, elevatorConstants.kI, elevatorConstants.kD));

    }

    public void activeMotor (boolean motorOn)
    {
        if (motorOn)
        {
            elevatorMotor.set(Constants.elevatorConstants.elevatorSpeed);
        }
        else
        {
            elevatorMotor.set(Constants.elevatorConstants.elevatorOff);
        }


}

}



