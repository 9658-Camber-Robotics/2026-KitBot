package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDS;
import frc.robot.Constants.Shooter;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.local.SparkWrapper;

public class ShooterSubsystem extends SubsystemBase
{

  private final SparkMax motorController = new SparkMax(CANIDS.shooterCANID, MotorType.kBrushless);
  private final SmartMotorController shooterMotorController = new SparkWrapper(motorController,
                                                                               Shooter.motor,
                                                                               Shooter.smc.clone().withSubsystem(this));
  private final FlyWheel             flyWheel               = new FlyWheel(Shooter.config.clone()
                                                                                         .withSmartMotorController(
                                                                                             shooterMotorController));

  public ShooterSubsystem()
  {
  }

  public void setDutycycle(double dutycycle)
  {
    flyWheel.setDutyCycleSetpoint(dutycycle);
  }

  public void setVelocity(AngularVelocity velocity)
  {
    flyWheel.setMechanismVelocitySetpoint(velocity);
  }

  public AngularVelocity getVelocity()
  {
    return flyWheel.getSpeed();
  }

  @Override
  public void simulationPeriodic()
  {
    flyWheel.simIterate();
  }

  @Override
  public void periodic()
  {
    flyWheel.updateTelemetry();
  }

  public Command setDutycycleCommand(double dutycycle)
  {
    return flyWheel.set(dutycycle);
  }

  public Command setVelocityCommand(AngularVelocity velocity)
  {
    return flyWheel.run(velocity);
  }
}