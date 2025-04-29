package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PIDmotor extends SubsystemBase {
    private SparkFlex testMotor;
    
    private ProfiledPIDController PID;

    private PIDmotor() {
        testMotor = new SparkFlex(1, SparkFlex.MotorType.kBrushless);

        PID = new ProfiledPIDController(0, 0, 0, null);
    }

    @Override
    public void periodic() {

    }
}
