// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.utils.FieldConstants.Hub;
import frc.robot.utils.FieldConstants.Outpost;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.gearing.Sprocket;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.FlyWheelConfig;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static class CANIDS
  {

    public static final int shooterCANID = 4;
    public static final int shooterCANIDtwo = 41;
    public static final int indexerCANID = 3;
    public static final int climbCANID   = 9;
    public static final int climbCANID2  = 0; // Ruh roh
  }

  public static class SwerveDrive 
  {

    public static final Distance robotWidth  = Inches.of(24);
    public static final Distance robotLength = Inches.of(32);
    public static final double maxSpeed = 9.2 * .5; // This still seems somewhat fast.
    public static final Pose2d   startPose   = new Pose2d(new Translation2d(Meter.of(3.5),
                                                                            Meter.of(4)),

                                                                          // red 13x 4y
                                                                          // blue 4x, 4y
                                                          Rotation2d.fromDegrees(0));
                                                          //red 180 
                                                          //blue 0

    public static class Setpoints
    {

      public static final Pose2d robotPoseAtOutpost = new Pose2d(Outpost.centerPoint, Rotation2d.kZero)
          .plus(new Transform2d(robotLength.div(2).in(Meters), 0, Rotation2d.kZero));
      public static final Pose2d robotPoseAtHub     = Hub.nearFace
          .plus(new Transform2d(robotLength.div(2).in(Meters), 0, Rotation2d.kZero));
    }
  }

  public static final TelemetryVerbosity verbosity = TelemetryVerbosity.HIGH;

  public static class Climber
  {

    public static class Setpoints
    {

      public static final Angle climbSetpoint   = Degrees.of(180);
      public static final Angle releaseSetpoint = Degrees.of(45);
    }

    public static final DCMotor                    motor = DCMotor.getKrakenX60(2);
    public static final SmartMotorControllerConfig smc   = new SmartMotorControllerConfig()
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withTelemetry("ClimberMotor", verbosity)
        .withIdleMode(MotorMode.BRAKE)
        .withMotorInverted(false)
        .withSupplyCurrentLimit(Amps.of(60))
        .withGearing(new MechanismGearing(GearBox.fromTeeth(20,10,28)))
        .withClosedLoopController(0, 0, 0)
        .withFeedforward(new ArmFeedforward(0, 0, 0))
        .withFollowers(Pair.of(new TalonFX(CANIDS.shooterCANIDtwo),false));

    public static final ArmConfig config = new ArmConfig()
        .withTelemetry("Climber", verbosity)
        .withHardLimit(Degrees.of(-180), Degrees.of(180))
        .withStartingPosition(Degrees.of(-180))
        .withLength(Inches.of(30))
        .withMass(Pounds.of(8));
  }

  public static class Indexer
  {

    public static final SmartMotorControllerConfig smc = new SmartMotorControllerConfig()
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withTelemetry("IndexerMotor", verbosity)
        .withIdleMode(MotorMode.BRAKE)
        .withMotorInverted(false)
        .withStatorCurrentLimit(Amps.of(40))
        .withGearing(new MechanismGearing(GearBox.fromTeeth(12,80,36)))
        .withClosedLoopController(0, 0, 0)
        .withFeedforward(new SimpleMotorFeedforward(0, 10, 0));

    public static final DCMotor        motor  = DCMotor.getNEO(1);
    public static final FlyWheelConfig config = new FlyWheelConfig()
        .withTelemetry("Indexer", verbosity)
        .withMass(Pounds.of(1))
        .withDiameter(Inches.of(2));

    public static class Setpoints
    {

      public static final AngularVelocity indexSpeed = RPM.of(1000);
    }
  }

  public static class Shooter
  {

    public static class Setpoints
    {

      public static final AngularVelocity tolerance = RPM.of(60); //60
      public static final AngularVelocity lowRPM    = RPM.of(1100); // 3100
      public static final AngularVelocity midRPM    = RPM.of(1500); // 3500
      public static final AngularVelocity high      = RPM.of(1600); // 3600
      public static final AngularVelocity maxRPM    = RPM.of(1600); // 3600
      public static final AngularVelocity autonomousPeriodRPM = RPM.of(3350);

    }

    public static final Debouncer                  flyWheelRecoveryDebouncer = new Debouncer(0.25,
                                                                                             DebounceType.kFalling);
    public static final DCMotor                    motor                     = DCMotor.getKrakenX60(2);
    public static final SmartMotorControllerConfig smc                       = new SmartMotorControllerConfig()
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withTelemetry("ShooterMotor", verbosity)
        .withIdleMode(MotorMode.COAST)
        .withMotorInverted(false) 
        .withFollowers(Pair.of(new TalonFX(41), true ) )
        .withStatorCurrentLimit(Amps.of(40))
        .withGearing(new MechanismGearing(GearBox.fromTeeth(40,60,60)))
        .withClosedLoopController(0.001, 0, 0)
        .withFeedforward(new SimpleMotorFeedforward(0.000, 0.170, 0.1));
    public static final FlyWheelConfig             config                    = new FlyWheelConfig()
        .withTelemetry("Shooter", verbosity)
        .withDiameter(Inches.of(4))
        .withMass(Pounds.of(2));
  }
// stuff so this will push

}
