package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ServoTest;

public class TestServo extends Command {
    private ServoTest servo;
    public boolean running90;

    public TestServo() {
        servo = ServoTest.getInstance();
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        running90 = true;
        servo.setServo(90);
        System.out.println("Turn 90");
        running90 = false;
    }
}