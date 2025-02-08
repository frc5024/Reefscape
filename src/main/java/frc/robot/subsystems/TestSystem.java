package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.TestCommand;

public class TestSystem extends SubsystemBase {
    private static TestSystem mInstance = null;

    public TestSystem() {

    }

    public static final TestSystem getInstance() {
        if (mInstance == null) {
            mInstance = new TestSystem();
        }
        return mInstance;
    }

    public Command testCommand() {
        return runOnce(() -> {
            new TestCommand();
        });
    }
}
