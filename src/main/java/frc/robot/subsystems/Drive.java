// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class Drive extends SubsystemBase {
  // The motors on the left side of the drive.
  private final CANSparkMax m_LeftFrontMotor = new CANSparkMax(DriveConstants.kLeftFrontMotor1Port, MotorType.kBrushless);
  private final CANSparkMax m_LeftBackMotor = new CANSparkMax(DriveConstants.kLeftBackMotor1Port, MotorType.kBrushless);
  // The motors on the right side of the drive.
  private final CANSparkMax m_RightFrontMotor = new CANSparkMax(DriveConstants.kRightFrontMotor1Port, MotorType.kBrushless);
  private final CANSparkMax m_RightBackMotor = new CANSparkMax(DriveConstants.kRightBackMotor1Port, MotorType.kBrushless);

  private final MotorControllerGroup m_LeftGroup = new MotorControllerGroup(m_LeftBackMotor, m_LeftFrontMotor);
  private final MotorControllerGroup m_RightGroup = new MotorControllerGroup(m_RightBackMotor, m_RightFrontMotor);

  // The robot's drive
  private final DifferentialDrive m_drive =
      new DifferentialDrive(m_LeftGroup, m_RightGroup);

  // The left-side drive encoder
  private final RelativeEncoder m_LeftFrontEncoder = m_LeftFrontMotor.getEncoder();
  private final RelativeEncoder m_LeftBackEncoder = m_LeftBackMotor.getEncoder();

  // The right-side drive encoder
  private final RelativeEncoder m_RightFrontEncoder = m_RightBackMotor.getEncoder();
  private final RelativeEncoder m_RightBackEncoder = m_RightBackMotor.getEncoder();


  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  // Create a new SysId routine for characterizing the drive.
  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motors.
              (Measure<Voltage> volts) -> {
                m_LeftFrontMotor.setVoltage(volts.in(Volts));
                m_LeftBackMotor.setVoltage(volts.in(Volts));
                m_RightFrontMotor.setVoltage(volts.in(Volts));
                m_RightBackMotor.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_LeftFrontMotor.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(m_LeftBackEncoder.getPosition() * DriveConstants.kWheelDiameterMeters * DriveConstants.kDriveMotorGearRatio, Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(m_LeftBackEncoder.getVelocity() * DriveConstants.kWheelDiameterMeters * DriveConstants.kDriveMotorGearRatio, MetersPerSecond));

                // Record a frame for the right motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-right")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_RightFrontMotor.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(m_RightFrontEncoder.getPosition() * DriveConstants.kWheelDiameterMeters * DriveConstants.kDriveMotorGearRatio, Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(m_RightBackEncoder.getVelocity() * DriveConstants.kWheelDiameterMeters * DriveConstants.kDriveMotorGearRatio, MetersPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));

  /** Creates a new Drive subsystem. */
  public Drive() {
      m_LeftFrontMotor.setInverted(true);
      m_RightFrontMotor.setInverted(true);
  }

  /**
   * Returns a command that drives the robot with arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public Command arcadeDriveCommand(DoubleSupplier fwd, DoubleSupplier rot) {
    // A split-stick arcade command, with forward/backward controlled by the left
    // hand, and turning controlled by the right.
    return run(() -> m_drive.arcadeDrive(fwd.getAsDouble(), rot.getAsDouble()))
        .withName("arcadeDrive");
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}
