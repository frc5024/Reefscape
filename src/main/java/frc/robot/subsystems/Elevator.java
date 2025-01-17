package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Elevator extends SubsystemBase{
    private TalonFX elevatorMotor;
    

    public Elevator() {
        elevatorMotor = new TalonFX(0);


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

public void reverseMotor (boolean reverseOn)
{
    if (reverseOn){
        elevatorMotor.set(-Constants.elevatorConstants.elevatorSpeed); 
    }
    else{
        elevatorMotor.set(Constants.elevatorConstants.elevatorOff);
    }

}


}



