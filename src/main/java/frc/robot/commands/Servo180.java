package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ServoTest;

public class Servo180 extends Command {
    private ServoTest servo;

    public Servo180() {
        servo = ServoTest.getInstance();
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        servo.setServo(180);
        System.out.println("Turn 180");
    }
}
