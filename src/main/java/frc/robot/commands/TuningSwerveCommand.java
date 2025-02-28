package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.TeleopConstants;
import frc.robot.autonomous.AutoBuilder;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.utils.LoggedTunableNumber;

/**
 * 
 */
public class TuningSwerveCommand extends Command {
    /* Subsystems */
    private final SwerveDriveSubsystem swerveDriveSubsystem;

    /* Suppliers */
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier omegaSupplier;
    private final CommandXboxController commandXboxController;

    /* Tunable PID for individual swerve modules */
    private LoggedTunableNumber driveKp;
    private LoggedTunableNumber driveKi;
    private LoggedTunableNumber driveKd;

    private LoggedTunableNumber turnKp;
    private LoggedTunableNumber turnKi;
    private LoggedTunableNumber turnKd;

    /* Tunable PID used for Path Planner autonomous mode in the DrivePathCommand */
    private LoggedTunableNumber xKp;
    private LoggedTunableNumber xKi;
    private LoggedTunableNumber xKd;

    private LoggedTunableNumber yKp;
    private LoggedTunableNumber yKi;
    private LoggedTunableNumber yKd;

    private LoggedTunableNumber omegaKp;
    private LoggedTunableNumber omegaKi;
    private LoggedTunableNumber omegaKd;

    /* Tunable PID for arm module */
    private LoggedTunableNumber elevatorKp;
    private LoggedTunableNumber elevatorKi;
    private LoggedTunableNumber elevatorKd;

    // Tunables used to set the robot initial Pose2d
    private LoggedTunableNumber xInitialPosition;
    private LoggedTunableNumber yInitialPosition;
    private LoggedTunableNumber rInitialAngle;

    // Tunables used to set the robot to drive to a particular Pose2d
    private LoggedTunableNumber xDriveToPosition;
    private LoggedTunableNumber yDriveToPosition;
    private LoggedTunableNumber rDriveToAngle;

    private LoggedNetworkBoolean driveBackAndForth;
    private LoggedNetworkBoolean driveSideToSide;
    private LoggedNetworkBoolean alternateRotation;
    private LoggedNetworkBoolean driveClosedLoop;

    // Tunables used to drive by set chassis speeds
    private LoggedTunableNumber vxMPS;
    private LoggedTunableNumber vyMPS;
    private LoggedTunableNumber omRPS;
    private LoggedTunableNumber angle;
    private LoggedNetworkBoolean driveByController;
    private LoggedNetworkBoolean driveByPathFinding;
    private LoggedNetworkBoolean driveByPosition;
    private LoggedNetworkBoolean driveByVelocities;

    // Tunable to set the arm subsystem to a particular angle
    private LoggedTunableNumber desiredHeight;

    // Tunable to set the shooter subsystem to a particular velocity
    // private final LoggedTunableNumber desiredRPM = new
    // LoggedTunableNumber("Shooter/DesiredRPM", 0.0);

    /* These are used for Path Planner autonomous mode in the DrivePathCommand */
    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;
    private final ProfiledPIDController omegaController;

    //
    private final double BACK_AND_FORTH_DISTANCE = 5.0;
    private final Timer timer;
    private double rotationalAngle;
    private boolean firstCall;

    /**
     * This command should only be run in robot tuning mode
     */
    public TuningSwerveCommand(SwerveDriveSubsystem swerveDriveSubsystem, DoubleSupplier xSupplier,
            DoubleSupplier ySupplier, DoubleSupplier omegaSupplier, CommandXboxController commandXboxController) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;

        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.omegaSupplier = omegaSupplier;
        this.commandXboxController = commandXboxController;

        double[] driveXPIDs = PIDConstants.getDriveXPIDs();
        double[] driveYPIDs = PIDConstants.getDriveXPIDs();
        double[] driveOmegaPIDs = PIDConstants.getDriveOmegaPIDs();

        this.xController = new ProfiledPIDController(driveXPIDs[0], driveXPIDs[1], driveXPIDs[2],
                TeleopConstants.X_CONSTRAINTS);
        this.yController = new ProfiledPIDController(driveYPIDs[0], driveYPIDs[1], driveYPIDs[2],
                TeleopConstants.Y_CONSTRAINTS);
        this.omegaController = new ProfiledPIDController(driveOmegaPIDs[0], driveOmegaPIDs[1], driveOmegaPIDs[2],
                TeleopConstants.OMEGA_CONSTRAINTS);

        this.xController.setTolerance(0.01);
        this.yController.setTolerance(0.01);
        this.omegaController.setTolerance(Units.degreesToRadians(1));
        this.omegaController.enableContinuousInput(-Math.PI, Math.PI);

        this.timer = new Timer();
        this.firstCall = true;
        this.rotationalAngle = 360.0; // this value is used to determine if the driverPov has been pressed

        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void end(boolean interrupted) {
        this.swerveDriveSubsystem.stop();

        Logger.recordOutput("Commands/Active Command", "");
    }

    @Override
    public void execute() {
        // Check for dashboard changes
        if (this.elevatorKp.hasChanged(hashCode()) || this.elevatorKi.hasChanged(hashCode())
                || this.elevatorKd.hasChanged(hashCode())) {
            // this.elevatorSubsystem.updatePID(this.elevatorKp.get(),
            // this.elevatorKi.get(), this.elevatorKd.get());
        }

        if (this.driveKp.hasChanged(hashCode()) || this.driveKi.hasChanged(hashCode())
                || this.driveKd.hasChanged(hashCode())) {
            this.swerveDriveSubsystem.updateDrivePID(this.driveKp.get(), this.driveKi.get(), this.driveKd.get());
        }

        if (this.turnKp.hasChanged(hashCode()) || this.turnKi.hasChanged(hashCode())
                || this.turnKd.hasChanged(hashCode())) {
            this.swerveDriveSubsystem.updateTurnPID(this.turnKp.get(), this.turnKi.get(), this.turnKd.get());
        }

        if (this.xKp.hasChanged(hashCode()) || this.xKi.hasChanged(hashCode()) || this.xKd.hasChanged(hashCode())) {
            this.xController.setPID(this.xKp.get(), this.xKi.get(), this.xKd.get());
        }

        if (this.yKp.hasChanged(hashCode()) || this.yKi.hasChanged(hashCode()) || this.yKd.hasChanged(hashCode())) {
            this.yController.setPID(this.yKp.get(), this.yKi.get(), this.yKd.get());
        }

        if (this.omegaKp.hasChanged(hashCode()) || this.omegaKi.hasChanged(hashCode())
                || this.omegaKd.hasChanged(hashCode())) {
            this.omegaController.setPID(this.omegaKp.get(), this.omegaKi.get(), this.omegaKd.get());
        }

        if (this.xInitialPosition.hasChanged(hashCode()) || this.yInitialPosition.hasChanged(hashCode())
                || this.rInitialAngle.hasChanged(hashCode())) {
            this.swerveDriveSubsystem.setPose(new Pose2d(this.xInitialPosition.get(), this.yInitialPosition.get(),
                    new Rotation2d(this.rInitialAngle.get())));
        }

        if (this.xDriveToPosition.hasChanged(hashCode()) || this.yDriveToPosition.hasChanged(hashCode())
                || this.rDriveToAngle.hasChanged(hashCode())) {
            this.xController.setGoal(this.xDriveToPosition.get());
            this.yController.setGoal(this.yDriveToPosition.get());
            this.omegaController.setGoal(Units.degreesToRadians(this.rDriveToAngle.get()));
        }

        if (this.desiredHeight.hasChanged(hashCode())) {
            // this.elevatorSubsystem.setGoal(this.desiredHeight.get());
        }

        // if (this.desiredRPM.hasChanged(hashCode())) {
        // this.coralSubsystem.setDesiredRPM(this.desiredRPM.get());
        // }

        if (this.driveByController.get()) {
            handleDriveByController();
        } else if (this.driveByPathFinding.get()) {
            if (this.firstCall) {
                this.xController.reset(this.swerveDriveSubsystem.getPose().getX());
                this.yController.reset(this.swerveDriveSubsystem.getPose().getY());
                this.omegaController.reset(this.swerveDriveSubsystem.getPose().getRotation().getRadians());

                this.firstCall = false;
            }

            Pose2d targetPose = new Pose2d(this.xDriveToPosition.get(), this.yDriveToPosition.get(),
                    new Rotation2d(this.rDriveToAngle.get()));
            handleDriveByPathFinding(targetPose);
        } else if (this.driveByPosition.get()) {
            if (this.firstCall) {
                this.xController.reset(this.swerveDriveSubsystem.getPose().getX());
                this.yController.reset(this.swerveDriveSubsystem.getPose().getY());
                this.omegaController.reset(this.swerveDriveSubsystem.getPose().getRotation().getRadians());

                this.firstCall = false;
            }

            handleAutonomousDriving();
        } else if (this.driveByVelocities.get()) {
            this.swerveDriveSubsystem.drive(this.vxMPS.get(), this.vyMPS.get(), this.omRPS.get(),
                    new Rotation2d(this.angle.get()), false);
        } else if (this.driveBackAndForth.get() || this.driveSideToSide.get() || this.alternateRotation.get()) {
            handleAutonomousDriving();
        } else {
            this.swerveDriveSubsystem.stop();
            this.firstCall = true;
            this.rotationalAngle = 360.0;
        }
    }

    @Override
    public void initialize() {
        setTunableNumbers();
        setBindings();
        resetPIDControllers();

        this.xController.reset(this.swerveDriveSubsystem.getPose().getX());
        this.yController.reset(this.swerveDriveSubsystem.getPose().getY());
        this.omegaController.reset(this.swerveDriveSubsystem.getPose().getRotation().getRadians());

        Logger.recordOutput("Commands/Active Command", this.getName());
    }

    /**
     * 
     */
    private void handleAutonomousDriving() {
        if (!this.timer.isRunning()) {
            this.timer.reset();
            this.timer.start();
        }

        if (this.driveBackAndForth.get()) {
            double xCurrent = this.swerveDriveSubsystem.getPose().getX();
            double xGoal = xCurrent >= 2.5 ? 0 : BACK_AND_FORTH_DISTANCE;
            this.xController.setGoal(xGoal);
        } else if (this.alternateRotation.get()) {
            double oCurrent = this.swerveDriveSubsystem.getPose().getRotation().getDegrees();
            double oGoal = oCurrent >= 89 ? 0 : 90;
            this.omegaController.setGoal(Units.degreesToRadians(oGoal));
        }

        if (isAtGoal()) {
            this.swerveDriveSubsystem.stop();
            this.timer.stop();
        } else if (this.timer.isRunning() && this.timer.hasElapsed(5.0)) {
            this.timer.stop();
        } else {
            Pose2d robotPose = this.swerveDriveSubsystem.getPose();

            double xSpeed = this.xController.calculate(robotPose.getX());
            double ySpeed = this.yController.calculate(robotPose.getY());
            double omegaSpeed = this.omegaController.calculate(robotPose.getRotation().getRadians());

            if (this.xController.atGoal())
                xSpeed = 0;
            if (this.yController.atGoal())
                ySpeed = 0;
            if (this.omegaController.atGoal())
                omegaSpeed = 0;

            this.swerveDriveSubsystem.drive(xSpeed, ySpeed, omegaSpeed, robotPose.getRotation(), false);
        }
    }

    /**
     * 
     */
    private void handleDriveByPathFinding(Pose2d targetPose) {
        Command pathFindingCommand = AutoBuilder.getPathFindingCommand(targetPose);

        Command commandGroup = Commands.sequence(
                new InstantCommand(() -> this.swerveDriveSubsystem.zeroDrivePID()),
                pathFindingCommand,
                new InstantCommand(() -> this.swerveDriveSubsystem.resetDrivePID()));
        commandGroup.schedule();
    }

    /**
     * 
     */
    private void handleDriveByController() {
        // Set controller variables
        Rotation2d angle = this.swerveDriveSubsystem.getPose().getRotation();

        Translation2d linearVelocity = SwerveDriveCommands.getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
                ySupplier.getAsDouble());

        // Apply rotation deadband
        double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), 0.1);

        // Square rotation value for more precise control
        omega = Math.copySign(omega * omega, omega);

        //
        if (omega == 0.0) {
            if (this.rotationalAngle != 360.0) {
                double goalRotation = Units.degreesToRadians(this.rotationalAngle);
                this.omegaController.setGoal(goalRotation);
                omega = this.omegaController.calculate(this.swerveDriveSubsystem.getRotation().getRadians());
            }

            if (this.omegaController.atGoal()) {
                omega = 0;
                this.rotationalAngle = 360.0;
            }
        }

        if (this.driveBackAndForth.get() || this.driveSideToSide.get() || this.alternateRotation.get()) {
            this.driveBackAndForth.set(false);
            this.driveSideToSide.set(false);
            this.alternateRotation.set(false);
        }

        // Convert to field relative speeds & send command
        ChassisSpeeds speeds = new ChassisSpeeds(
                linearVelocity.getX() * SwerveConstants.maxLinearSpeed,
                linearVelocity.getY() * SwerveConstants.maxLinearSpeed,
                omega * SwerveConstants.maxAngularSpeed);

        boolean isFlipped = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;

        swerveDriveSubsystem.drive(linearVelocity.getX(), linearVelocity.getY(), omega);
    }

    /**
     * 
     */
    private boolean isAtGoal() {
        return this.xController.atGoal() && this.yController.atGoal() && this.omegaController.atGoal();
    }

    /**
     * 
     */
    private void resetPIDControllers() {
        Pose2d robotPose = this.swerveDriveSubsystem.getPose();

        this.xController.reset(robotPose.getX());
        this.yController.reset(robotPose.getY());
        this.omegaController.reset(robotPose.getRotation().getRadians());
    }

    /**
     * 
     */
    private void setBindings() {
        this.commandXboxController.povUp().onTrue(runOnce(() -> this.rotationalAngle = 0.0));
        this.commandXboxController.povRight().onTrue(runOnce(() -> this.rotationalAngle = -90.0));
        this.commandXboxController.povDown().onTrue(runOnce(() -> this.rotationalAngle = -180.0));
        this.commandXboxController.povLeft().onTrue(runOnce(() -> this.rotationalAngle = 90.0));
    }

    /**
     * 
     */
    private void setTunableNumbers() {
        double[] drivePIDs = PIDConstants.getDrivePIDs();
        double[] turnPIDs = PIDConstants.getTurnPIDs();

        // LoggedTunableNumbers are logged under /Tuning
        this.driveKp = new LoggedTunableNumber("SwerveDriveModule/Drive/kp", drivePIDs[0]);
        this.driveKi = new LoggedTunableNumber("SwerveDriveModule/Drive/ki", drivePIDs[1]);
        this.driveKd = new LoggedTunableNumber("SwerveDriveModule/Drive/kd", drivePIDs[2]);

        this.turnKp = new LoggedTunableNumber("SwerveDriveModule/Turn/kp", turnPIDs[0]);
        this.turnKi = new LoggedTunableNumber("SwerveDriveModule/Turn/ki", turnPIDs[1]);
        this.turnKd = new LoggedTunableNumber("SwerveDriveModule/Turn/kd", turnPIDs[2]);

        double[] driveXPIDs = PIDConstants.getDriveXPIDs();
        double[] driveYPIDs = PIDConstants.getDriveXPIDs();
        double[] driveOmegaPIDs = PIDConstants.getDriveOmegaPIDs();

        this.xKp = new LoggedTunableNumber("PathPlanner/xKp", driveXPIDs[0]);
        this.xKi = new LoggedTunableNumber("PathPlanner/xKi", driveXPIDs[1]);
        this.xKd = new LoggedTunableNumber("PathPlanner/xKd", driveXPIDs[2]);

        this.yKp = new LoggedTunableNumber("PathPlanner/yKp", driveYPIDs[0]);
        this.yKi = new LoggedTunableNumber("PathPlanner/yKi", driveYPIDs[1]);
        this.yKd = new LoggedTunableNumber("PathPlanner/yKd", driveYPIDs[2]);

        this.omegaKp = new LoggedTunableNumber("PathPlanner/omegaKp", driveOmegaPIDs[0]);
        this.omegaKi = new LoggedTunableNumber("PathPlanner/omegaKi", driveOmegaPIDs[1]);
        this.omegaKd = new LoggedTunableNumber("PathPlanner/omegaKd", driveOmegaPIDs[2]);

        double[] elevatorPIDs = PIDConstants.getElevatorPIDs();

        this.elevatorKp = new LoggedTunableNumber("Elevator/kp", elevatorPIDs[0]);
        this.elevatorKi = new LoggedTunableNumber("Elevator/ki", elevatorPIDs[1]);
        this.elevatorKd = new LoggedTunableNumber("Elevator/kd", elevatorPIDs[2]);
        this.desiredHeight = new LoggedTunableNumber("Elevator/DesiredHeight", 0.0);

        Pose2d currentPose = this.swerveDriveSubsystem.getPose();
        this.xInitialPosition = new LoggedTunableNumber("InitialPosition/xPosition", currentPose.getX());
        this.yInitialPosition = new LoggedTunableNumber("InitialPosition/yPosition", currentPose.getY());
        this.rInitialAngle = new LoggedTunableNumber("InitialPosition/rAngle",
                currentPose.getRotation().getDegrees());

        this.xDriveToPosition = new LoggedTunableNumber("DriveToPosition/xPosition", currentPose.getX());
        this.yDriveToPosition = new LoggedTunableNumber("DriveToPosition/yPosition", currentPose.getY());
        this.rDriveToAngle = new LoggedTunableNumber("DriveToPosition/rAngle",
                currentPose.getRotation().getDegrees());

        // LoggedNetworkBoolean don't prefix the network tables key
        this.driveBackAndForth = new LoggedNetworkBoolean("Tuning/Robot/Toggles/Drive Back and Forth", false);
        this.driveSideToSide = new LoggedNetworkBoolean("Tuning/Robot/Toggles/Drive Side to Side", false);
        this.alternateRotation = new LoggedNetworkBoolean("Tuning/Robot/Toggles/Alternate Rotation", false);
        this.driveClosedLoop = new LoggedNetworkBoolean("Tuning/Robot/Toggles/Drive Closed Loop", false);

        this.vxMPS = new LoggedTunableNumber("Robot/Chassis/vxMPS", 0.0);
        this.vyMPS = new LoggedTunableNumber("Robot/Chassis/vyMPS", 0.0);
        this.omRPS = new LoggedTunableNumber("Robot/Chassis/omRPS", 0.0);
        this.angle = new LoggedTunableNumber("Robot/Chassis/Angle", 0.0);

        this.driveByController = new LoggedNetworkBoolean("Tuning/Robot/Toggles/Drive By Controller", false);
        this.driveByPathFinding = new LoggedNetworkBoolean("Tuning/Robot/Toggles/Drive By Path Finding", false);
        this.driveByPosition = new LoggedNetworkBoolean("Tuning/Robot/Toggles/Drive By Position", false);
        this.driveByVelocities = new LoggedNetworkBoolean("Tuning/Robot/Toggles/Drive By Velocities", false);
    }
}
