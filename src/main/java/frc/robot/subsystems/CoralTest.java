package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralTest extends SubsystemBase {
    private SparkFlex motor;

    public CoralTest() {
        motor = new SparkFlex(51, SparkFlex.MotorType.kBrushless);
    }

    public void start() {
        motor.set(0.3);
        // System.out.println("AAAAAAAAAAAAAAAAAAAAAAAAAAAa");
    }

    public void stop() {
        motor.set(0);
        // System.out.println("EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE");
    }
}
