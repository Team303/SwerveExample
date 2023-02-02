// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap.SwerveConstants;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
    private final ShuffleboardTab tab = Shuffleboard.getTab("Swerve");

    private final ShuffleboardLayout frontLeftLayout = tab.getLayout("Front Left", "grid");
    private final ShuffleboardLayout frontRightLayout = tab.getLayout("Front Right", "grid");
    private final ShuffleboardLayout backLeftLayout = tab.getLayout("Back Left", "grid");
    private final ShuffleboardLayout backRightLayout = tab.getLayout("Back Right", "grid");

    public static final double kMaxSpeed = 3.0; // 3 meters per second
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

    private final Translation2d m_frontLeftLocation = new Translation2d(SwerveConstants.TRACKWIDTH,
            SwerveConstants.TRACKWIDTH);
    private final Translation2d m_frontRightLocation = new Translation2d(SwerveConstants.TRACKWIDTH,
            -SwerveConstants.TRACKWIDTH);
    private final Translation2d m_backLeftLocation = new Translation2d(-SwerveConstants.TRACKWIDTH,
            SwerveConstants.TRACKWIDTH);
    private final Translation2d m_backRightLocation = new Translation2d(-SwerveConstants.TRACKWIDTH,
            -SwerveConstants.TRACKWIDTH);

    private final SwerveModule m_frontLeft = new SwerveModule(SwerveConstants.LEFT_FRONT_DRIVE_ID,
            SwerveConstants.LEFT_FRONT_STEER_ID, SwerveConstants.LEFT_FRONT_STEER_CANCODER_ID,
            SwerveConstants.LEFT_FRONT_STEER_OFFSET, frontLeftLayout, false, false);
    private final SwerveModule m_frontRight = new SwerveModule(SwerveConstants.RIGHT_FRONT_DRIVE_ID,
            SwerveConstants.RIGHT_FRONT_STEER_ID, SwerveConstants.RIGHT_FRONT_STEER_CANCODER_ID,
            SwerveConstants.RIGHT_FRONT_STEER_OFFSET, frontRightLayout, true, false);
    private final SwerveModule m_backLeft = new SwerveModule(SwerveConstants.LEFT_BACK_DRIVE_ID,
            SwerveConstants.LEFT_BACK_STEER_ID, SwerveConstants.LEFT_BACK_STEER_CANCODER_ID,
            SwerveConstants.LEFT_BACK_STEER_OFFSET, backLeftLayout, false, false);
    private final SwerveModule m_backRight = new SwerveModule(SwerveConstants.RIGHT_BACK_DRIVE_ID,
            SwerveConstants.RIGHT_BACK_STEER_ID, SwerveConstants.RIGHT_BACK_STEER_CANCODER_ID,
            SwerveConstants.RIGHT_BACK_STEER_OFFSET, backRightLayout, true, false);

    private final AHRS m_gyro = new AHRS();

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
            m_kinematics,
            m_gyro.getRotation2d(),
            new SwerveModulePosition[] {
                    m_frontLeft.getPosition(),
                    m_frontRight.getPosition(),
                    m_backLeft.getPosition(),
                    m_backRight.getPosition()
            });

    public Drivetrain() {
        m_gyro.reset();
    }

    public void zeroGyro() {
        this.m_gyro.reset();
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        var swerveModuleStates = m_kinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
    }

    public void periodic() {
        m_frontLeft.periodic();
        m_frontRight.periodic();
        m_backLeft.periodic();
        m_backRight.periodic();
    }

    public void teleopPeriodic() {
        m_frontLeft.teleopPeriodic();
        m_frontRight.teleopPeriodic();
        m_backLeft.teleopPeriodic();
        m_backRight.teleopPeriodic();
    }

    /** Updates the field relative position of the robot. */
    public void updateOdometry() {
        m_odometry.update(
                m_gyro.getRotation2d(),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_backLeft.getPosition(),
                        m_backRight.getPosition()
                });
    }
}
