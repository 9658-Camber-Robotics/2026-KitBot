package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANIDS;
import frc.robot.Constants.Shooter;

import java.lang.constant.Constable;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ShooterSubsystem extends SubsystemBase
{

  private final TalonFX             motorController = new TalonFX(CANIDS.shooterCANID);
  private final SmartMotorController shooterMotorController;
  private final FlyWheel             flyWheel;

  public ShooterSubsystem()
  {
    shooterMotorController = new TalonFXWrapper(motorController, Shooter.motor, Shooter.smc.withSubsystem(this));
    flyWheel = new FlyWheel(Shooter.config.withSmartMotorController(shooterMotorController));
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

  public Command setVelocityCommand(Supplier<AngularVelocity> velocity)
  {
    return flyWheel.run(velocity);
  }

    public boolean isNear(AngularVelocity tolerance) {

      return Shooter.flyWheelRecoveryDebouncer.calculate(flyWheel.getSpeed().isNear(shooterMotorController.getMechanismSetpointVelocity().orElse(shooterMotorController.getMechanismVelocity()),tolerance));
    }
}