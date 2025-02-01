package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ServoTest;

public class TestServo extends Command {
    private ServoTest servo;

    public TestServo() {
        servo = ServoTest.getInstance();
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        servo.setServo(90);// Testing sets to an angle
        // System.out.println("Turn 90");

    }
}