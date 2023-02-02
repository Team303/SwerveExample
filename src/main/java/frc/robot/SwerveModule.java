// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
  private static final double kWheelRadius = 0.10033;
  private static final int kEncoderResolution = 4096;

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final CANCoder m_turningEncoder;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_turningPIDController = new PIDController(
      0.08,
      0.0625,
      0.001);

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);

  private SwerveModuleState desiredState = new SwerveModuleState(0,
      Rotation2d.fromDegrees(SmartDashboard.getNumber("Desired Angle", 180)));

  private double desiredTurnAngle;
  private double turnOutput;
  private double turnFeedforward;
  private double driveOutput;
  private double driveFeedforward;
  private double turnPositionError;
  private double turnVelocityError;

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
   * and turning encoder.
   *
   * @param driveMotorId   PWM output for the drive motor.
   * @param turningMotorId PWM output for the turning motor.
   * 
   */
  public SwerveModule(
      int driveMotorId,
      int turningMotorId,
      int steerEncoderId,
      double magnetOffset,
      ShuffleboardLayout layout,
      boolean driveInverted,
      boolean turnInverted
      ) {
    m_driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
    m_driveMotor.setInverted(driveInverted);

    m_turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);
    m_turningMotor.setInverted(turnInverted);

    m_driveEncoder = m_driveMotor.getEncoder();

    m_driveEncoder.setPositionConversionFactor(
        Math.PI * kWheelRadius * (1 / kEncoderResolution) * (1 / ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0))));
    m_driveEncoder.setVelocityConversionFactor(
        Math.PI * kWheelRadius * (1 / kEncoderResolution) * (1 / ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0))));

    m_turningEncoder = new CANCoder(steerEncoderId);

    m_turningEncoder.configMagnetOffset(magnetOffset);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(0, 360);
    m_turningPIDController.setTolerance(1.0);

    layout.addNumber("Position Distance", () -> this.getPosition().distanceMeters);
    layout.addNumber("Position Angle", () -> this.m_turningEncoder.getAbsolutePosition());
    layout.addNumber("Desired Turn Angle", () -> this.desiredTurnAngle);
    layout.addNumber("Turning Output", () -> this.turnOutput);
    layout.addNumber("Turning Feedforward", () -> this.turnFeedforward);
    layout.addNumber("Turning Position Error", () -> this.turnPositionError);
    layout.addNumber("Turning Velocity Error", () -> this.turnVelocityError);

    layout.add(m_turningPIDController);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), new Rotation2d(Math.toRadians(m_turningEncoder.getAbsolutePosition())));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(), new Rotation2d(Math.toRadians(m_turningEncoder.getAbsolutePosition())));
  }

  private static SwerveModuleState optimizeTurnAngle(SwerveModuleState desiredState, Rotation2d currentAngle) {
    Rotation2d delta = normalizeAngle180(desiredState.angle.minus(currentAngle));

    if (Math.abs(delta.getDegrees()) > 90.0) {
      return new SwerveModuleState(
          -desiredState.speedMetersPerSecond,
          normalizeAngle360(desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0))));
    } else {
      return new SwerveModuleState(desiredState.speedMetersPerSecond, normalizeAngle360(desiredState.angle));
    }
  }

  private static Rotation2d normalizeAngle180(Rotation2d rot) {
    double angle = rot.getDegrees();

    while (angle > 180)
      angle -= 360;
    while (angle < -180)
      angle += 360;

    return Rotation2d.fromDegrees(angle);
  }

  private static Rotation2d normalizeAngle360(Rotation2d rot) {
    double angle = rot.getDegrees();

    while (angle > 360)
      angle -= 360;
    while (angle < 0)
      angle += 360;

    return Rotation2d.fromDegrees(angle);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    this.desiredState = desiredState;
  }

  public void periodic() {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = optimizeTurnAngle(desiredState,
        Rotation2d.fromDegrees(m_turningEncoder.getAbsolutePosition()));

    // Calculate the drive output from the drive PID controller.
    this.driveOutput = state.speedMetersPerSecond;// m_drivePIDController.calculate(m_driveEncoder.getVelocity(), state.speedMetersPerSecond);

    this.driveFeedforward = 0;//m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_turningPIDController.calculate(m_turningEncoder.getAbsolutePosition(),
        state.angle.getDegrees());

    // final double turnFeedforward =
    // m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    this.desiredTurnAngle = state.angle.getDegrees();
    this.turnOutput = turnOutput;
    // this.turnFeedforward = turnFeedforward;
    this.turnPositionError = m_turningPIDController.getPositionError();
    this.turnVelocityError = m_turningPIDController.getVelocityError();
  }

  public void teleopPeriodic() {
    m_driveMotor.set((this.driveOutput) * 1);
    m_turningMotor.set(map(-this.turnOutput, -10, 10, -1, 1));
  }

  private static double map(double input, double startLower, double startUpper, double endLower, double endUpper) {
    return endLower + ((input - startLower) * (endUpper - endLower)) / (startUpper - startLower);
  }
}
