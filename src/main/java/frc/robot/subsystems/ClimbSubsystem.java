package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDS;
import frc.robot.Constants.Climber;
import frc.robot.Constants.Climber.Setpoints;
import java.util.function.Supplier;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.local.SparkWrapper;

public class ClimbSubsystem extends SubsystemBase
{


  // Vendor motor controller object
  private SparkMax spark = new SparkMax(CANIDS.climbCANID, MotorType.kBrushless);

  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController sparkSmartMotorController = new SparkWrapper(spark,
                                                                            Climber.motor,
                                                                            Climber.smc.clone()
                                                                                       .withSubsystem(this));

  // Arm Mechanism
  private Arm climb = new Arm(Climber.config.clone().withSmartMotorController(sparkSmartMotorController));

  public void setPosition(Angle position)
  {
    climb.setMechanismPositionSetpoint(position);
  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
    climb.updateTelemetry();
  }

  @Override
  public void simulationPeriodic()
  {
    // This method will be called once per scheduler run during simulation
    climb.simIterate();
  }

  public Command setDutycyleCommand(Supplier<Double> dutycycle)
  {
    return climb.set(dutycycle);
  }

  public Command setAngleCommand(Angle angle)
  {
    return climb.run(angle);
  }
}
