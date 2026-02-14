// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import yams.math.ExponentialProfilePIDController;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

import static edu.wpi.first.units.Units.*;

public class ElevatorSubsystem extends SubsystemBase {
    DCMotor motor = DCMotor.getNEO(1);
    double kP = ElevatorConstants.Kp;
    double kI = ElevatorConstants.Ki;
    double kD = ElevatorConstants.Kd;
    private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            // Mechanism Circumference is the distance traveled by each mechanism rotation converting rotations to meters.
            .withMechanismCircumference((ElevatorConstants.DrumRadius).times(2*Math.PI))
            // Feedback Constants (PID Constants)
            .withClosedLoopController(new ExponentialProfilePIDController(kP,kI,kD,
                    ExponentialProfilePIDController.createElevatorConstraints(ElevatorConstants.MaxVolts,
                            motor, ElevatorConstants.CarriageMass, ElevatorConstants.DrumRadius, ElevatorConstants.ElevatorGearBox)))
            // Feedforward Constants
            .withFeedforward(new ElevatorFeedforward(0, 0, 0))
            .withSimFeedforward(new ElevatorFeedforward(0, 0, 0))
            // Telemetry name and verbosity level
            .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
            // Gearing from the motor rotor to final shaft.
            // In this example gearbox(3,4) is the same as gearbox("3:1","4:1") which corresponds to the gearbox attached to your motor.
            .withGearing(ElevatorConstants.ElevatorGearBox)
            // Motor properties to prevent over currenting.
            .withMotorInverted(false)
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(Amps.of(40))
            .withClosedLoopRampRate(Seconds.of(0.25))
            .withOpenLoopRampRate(Seconds.of(0.25));

    private SparkMax spark = new SparkMax(1, MotorType.kBrushless);

    private SmartMotorController sparkSmartMotorController = new SparkWrapper(spark, motor, smcConfig);
    private ElevatorConfig elevconfig = new ElevatorConfig(sparkSmartMotorController)
            .withStartingHeight(ElevatorConstants.StartingHeight)
            .withHardLimits(ElevatorConstants.MinElevatorHeightMeters, ElevatorConstants.MaxElevatorHeightMeters)
            .withTelemetry("Elevator", TelemetryVerbosity.HIGH)
            .withMass((ElevatorConstants.CarriageMass));

    private Elevator elevator = new Elevator(elevconfig);

    public Command setHeight(Distance height) { return elevator.setHeight(height);}

    public Command resetHeight() { return elevator.setHeight(ElevatorConstants.StartingHeight);}

    public Command set(double dutycycle) { return elevator.set(dutycycle);}
    public Command sysId() { return elevator.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));}

    public ElevatorSubsystem() {
        SmartDashboard.putData(getName(), this);
    }
    @Override
    public void periodic(){
        elevator.updateTelemetry();
    }
    @Override
    public void simulationPeriodic() {
        elevator.simIterate();
    }
}
