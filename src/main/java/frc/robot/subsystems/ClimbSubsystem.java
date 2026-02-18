package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ClimbSubsystem extends SubsystemBase {
    double kP = ClimbConstants.ClimbKp;
    double kI = ClimbConstants.ClimbKi;
    double kD = ClimbConstants.ClimbKd;

    private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(kP, kI, kD)
            .withFeedforward(new ArmFeedforward(kP, kI, kD))
            .withSimFeedforward(new ArmFeedforward(kP, kI, kD))
            .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH)
            .withGearing(ClimbConstants.ClimbGearBox)
            .withMotorInverted(false)
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(Amps.of(40))
            .withClosedLoopRampRate(Seconds.of(0.25))
            .withOpenLoopRampRate(Seconds.of(0.25));

    // Vendor motor controller object
    private SparkMax spark = new SparkMax(1, MotorType.kBrushless);

    // Create our SmartMotorController from our Spark and config with the NEO.
    private SmartMotorController sparkSmartMotorController = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);

    private ArmConfig armCfg = new ArmConfig(sparkSmartMotorController)
            // Soft limit is applied to the SmartMotorControllers PID
            .withSoftLimits(ClimbConstants.softMinAngle, ClimbConstants.softMaxAngle)
            // Hard limit is applied to the simulation.
            .withHardLimit(ClimbConstants.hardMinAngle, ClimbConstants.hardMaxAngle)
            // Starting position is where your arm starts
            .withStartingPosition(ClimbConstants.startingAngle)
            // Length and mass of your arm for sim.
            .withLength(ClimbConstants.ClimbLength)
            .withMass(ClimbConstants.ClimbMass)
            // Telemetry name and verbosity for the arm.
            .withTelemetry("Arm", TelemetryVerbosity.HIGH);

    // Arm Mechanism
    private Arm arm = new Arm(armCfg);

    public Command setAngle(Angle angle) { return arm.setAngle(angle);}

    public Command resetAngle() { return setAngle(ClimbConstants.startingAngle);}
    public Command armL2() { return setAngle(ClimbConstants.L2Angle).withName("L2");}
    public Command armL3() { return setAngle(ClimbConstants.L3Angle).withName("L3");}
    public Command armIntake() { return setAngle(ClimbConstants.intakeAngle).withName("intake");}

    public Command set(double dutycycle) { return arm.set(dutycycle);}

    public Command sysId() { return arm.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));}

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        arm.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        arm.simIterate();
    }
}
