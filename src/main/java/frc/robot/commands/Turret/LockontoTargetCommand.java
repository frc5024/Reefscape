package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LockontoTargetCommand extends Command {

    private final Turret turretSubsystem; 
    private final Limelight limelightSubsystem;

    public LockontoTargetCommand(Turret turretSubsystem, Limelight limelightSubsystem, double robotX, double robotY, double targetX, double targetY) {
        this.turretSubsystem = turretSubsystem;
        this.limelightSubsystem = limelightSubsystem;

        addRequirements(turretSubsystem);
    }

    @Override
    public void initialize() {

        var limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        double tv = limelightTable.getEntry("tv").getDouble(0.0);
        double tx = limelightTable.getEntry("tx").getDouble(0.0);

        if(tv == 1) {
            double robotHeading = turretSubsystem.getRobotHeading();

            double turretTargetAngle = robotHeading + tx;

            turretSubsystem.setTargetAngle(turretTargetAngle);
        } else {
            turretSubsystem.setTargetAngle(turretSubsystem.getTurretAngle());
        }

    }

    @Override
    public boolean isFinished() {
        // Finish when the turret is locked to the target angle
        return turretSubsystem.isAtTargetAngle();
    }
}
