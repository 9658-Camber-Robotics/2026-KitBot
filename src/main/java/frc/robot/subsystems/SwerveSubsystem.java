// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.FieldConstants.Hub;
import limelight.Limelight;
import limelight.networktables.*;
import org.json.simple.parser.ParseException;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

public class SwerveSubsystem extends SubsystemBase {

    File directory = new File(Filesystem.getDeployDirectory(), "swerve");
    SwerveDrive swerveDrive;
    // limelight stuff
    private Limelight limelight_swerve;
    private LimelightPoseEstimator limelightPoseEstimator_swerve;
    private double lastLLTimestamp_swerve = 0;

    public SwerveSubsystem() {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.SwerveDrive.maxSpeed,
                    Constants.SwerveDrive.startPose);
            // Alternative method if you don't want to supply the conversion factor via JSON files.
            // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        swerveDrive.setModuleStateOptimization(true);
        setupPathPlanner();
        setupLimeLight();
    }

    public void setupLimeLight() {
        swerveDrive.stopOdometryThread();
        limelight_swerve = new Limelight("limelight");
        limelight_swerve
                .getSettings()
                .withPipelineIndex(0)
                .withCameraOffset(
                        new Pose3d(
                                Units.inchesToMeters(-9),
                                Units.inchesToMeters(-11),
                                Units.inchesToMeters(15),
                                new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(25), Units.degreesToRadians(0))))
                .withAprilTagIdFilter(List.of(17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11))
                .save();

        limelightPoseEstimator_swerve = limelight_swerve.createPoseEstimator(LimelightPoseEstimator.EstimationMode.MEGATAG1);

    }

    public double updateLimelight(Limelight ll, LimelightPoseEstimator llPoseEst, double llTImestamp, Angle cameraYaw, String llname) {
        ll
                .getSettings()
                .withRobotOrientation(
                        new Orientation3d(
                                new Rotation3d(swerveDrive.getOdometryHeading().rotateBy(new Rotation2d(cameraYaw))),
                                new AngularVelocity3d(DegreesPerSecond.of(0), DegreesPerSecond.of(0), DegreesPerSecond.of(0))))
                .save();
        Optional<PoseEstimate> poseEstimates =
                llPoseEst.getPoseEstimate();
        Optional<LimelightResults> results = ll.getLatestResults();
        if (results.isPresent() && poseEstimates.isPresent()) {
            LimelightResults result = results.get();
            PoseEstimate poseEstimate = poseEstimates.get();
            if (result.valid) {
                Pose2d estimatorPose = poseEstimate.pose.toPose2d();
                Pose2d usefulPose = result.getBotPose2d(Alliance.Blue);
                swerveDrive.field.getObject("Vision").setPose(estimatorPose);

                SmartDashboard.putNumber("LimeLightTuning/" + llname + "/ambiguity", poseEstimate.getAvgTagAmbiguity());
                if (poseEstimate.getAvgTagAmbiguity() < 0.1 &&
                        poseEstimate.tagCount > 1) {
                    if (llTImestamp != poseEstimate.timestampSeconds) {
                        var stdDevScale = Math.pow(poseEstimate.avgTagDist, 2.0) / poseEstimate.tagCount;

                        swerveDrive.addVisionMeasurement(estimatorPose,
                                poseEstimate.timestampSeconds);
                        return poseEstimate.timestampSeconds;
                    }
                }
            }
        }
        return llTImestamp;
    }

    @Override
    public void periodic() {
        swerveDrive.updateOdometry();
        lastLLTimestamp_swerve = updateLimelight(limelight_swerve, limelightPoseEstimator_swerve, lastLLTimestamp_swerve, Degrees.of(0), "swerve");
        SmartDashboard.putNumber("HubDistance(Meters)", distanceFromHubMeters());
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }

    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return run(() -> {
            swerveDrive.driveFieldOriented(velocity.get());
        });
    }

    /**
     * Setup AutoBuilder for PathPlanner.
     */
    public void setupPathPlanner() {
        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();

            final boolean enableFeedforward = true;
            // Configure AutoBuilder last
            AutoBuilder.configure(
                    swerveDrive::getPose,
                    // Robot pose supplier
                    swerveDrive::resetOdometry,
                    // Method to reset odometry (will be called if your auto has a starting pose)
                    swerveDrive::getRobotVelocity,
                    // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    (speedsRobotRelative, moduleFeedForwards) -> {
                        if (enableFeedforward) {
                            swerveDrive.drive(
                                    speedsRobotRelative,
                                    swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                                    moduleFeedForwards.linearForces()
                            );
                        } else {
                            swerveDrive.setChassisSpeeds(speedsRobotRelative);
                        }
                    },
                    // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also, optionally outputs individual module feedforwards
                    new PPHolonomicDriveController(
                            // PPHolonomicController is the built-in path following controller for holonomic drive trains
                            new PIDConstants(5.0, 0.0, 0.0),
                            // Translation PID constants
                            new PIDConstants(5.0, 0.0, 0.0)
                            // Rotation PID constants
                    ),
                    config,
                    // The robot configuration
                    () -> {
                        // Boolean supplier that controls when the path will be mirrored for the red alliance
                        // This will flip the path being followed to the red side of the field.
                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this
                    // Reference to this subsystem to set requirements
            );

        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }
    }

    /**
     * Get the path follower with events.
     *
     * @param pathName PathPlanner path name.
     * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
     */
    public Command getAutonomousCommand(String pathName) {
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return new PathPlannerAuto(pathName);
    }

    public Command driveToPose(Pose2d pose) {
// Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                swerveDrive.getMaximumChassisVelocity(), 4.0,
                swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

// Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindToPose(
                pose,
                constraints,
                edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
        );
    }

    /**
     * Drive with {@link SwerveSetpointGenerator} from 254, implemented by PathPlanner.
     *
     * @param robotRelativeChassisSpeed Robot relative {@link ChassisSpeeds} to achieve.
     * @return {@link Command} to run.
     * @throws IOException    If the PathPlanner GUI settings is invalid
     * @throws ParseException If PathPlanner GUI settings is nonexistent.
     */
    private Command driveWithSetpointGenerator(Supplier<ChassisSpeeds> robotRelativeChassisSpeed)
            throws IOException, ParseException {
        SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(RobotConfig.fromGUISettings(),
                swerveDrive.getMaximumChassisAngularVelocity());
        AtomicReference<SwerveSetpoint> prevSetpoint
                = new AtomicReference<>(new SwerveSetpoint(swerveDrive.getRobotVelocity(),
                swerveDrive.getStates(),
                DriveFeedforwards.zeros(swerveDrive.getModules().length)));
        AtomicReference<Double> previousTime = new AtomicReference<>();

        return startRun(() -> previousTime.set(Timer.getFPGATimestamp()),
                () -> {
                    double newTime = Timer.getFPGATimestamp();
                    SwerveSetpoint newSetpoint = setpointGenerator.generateSetpoint(prevSetpoint.get(),
                            robotRelativeChassisSpeed.get(),
                            newTime - previousTime.get());
                    swerveDrive.drive(newSetpoint.robotRelativeSpeeds(),
                            newSetpoint.moduleStates(),
                            newSetpoint.feedforwards().linearForces());
                    prevSetpoint.set(newSetpoint);
                    previousTime.set(newTime);

                });
    }

    public Command driveWithSetpointGeneratorFieldRelative(Supplier<ChassisSpeeds> fieldRelativeSpeeds) {
        try {
            return driveWithSetpointGenerator(() -> {
                return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds.get(), getHeading());

            });
        } catch (Exception e) {
            DriverStation.reportError(e.toString(), true);
        }
        return Commands.none();

    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /**
     * Gets the current pose (position and rotation) of the robot, as reported by odometry.
     *
     * @return The robot's pose
     */
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }


    public Command zeroGyroWithAllianceCommand() {
        return runOnce(() -> {
            DriverStation.getAlliance().ifPresent(alliance -> {
                swerveDrive.zeroGyro();
                if (alliance == DriverStation.Alliance.Red) {
                    swerveDrive.resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.k180deg));
                }
            });
        });
    }

    public Command resetOdometryCommand(Pose2d odom) {
        return runOnce(() -> swerveDrive.resetOdometry(AllianceFlipUtil.apply(odom)));
    }

    public Command lock() {
        return run(swerveDrive::lockPose);
    }

    public double distanceFromHubMeters() {
        return getPose().getTranslation().getDistance(AllianceFlipUtil.apply(Hub.topCenterPoint.toTranslation2d()));
    }

    public void driveFieldOrientedSetpoint(ChassisSpeeds speeds) {
        swerveDrive.driveFieldOriented(speeds);
    }
}
