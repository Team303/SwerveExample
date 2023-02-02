// Copyright (c) 2022 Team 303

package frc.robot;

public final class RobotMap {

	public static final class SwerveConstants {

		/* CAN IDs of Drive Motors */
		public static final int LEFT_FRONT_DRIVE_ID = 6;
		public static final int LEFT_BACK_DRIVE_ID = 3;
		public static final int RIGHT_FRONT_DRIVE_ID = 9;
		public static final int RIGHT_BACK_DRIVE_ID = 12;

		/* CAN IDs of steer Motors */
		public static final int LEFT_FRONT_STEER_ID = 4;
		public static final int LEFT_BACK_STEER_ID = 13;
		public static final int RIGHT_FRONT_STEER_ID = 7;
		public static final int RIGHT_BACK_STEER_ID = 10;

		/* Steer Encoder CAN IDs */
		public static final int LEFT_FRONT_STEER_CANCODER_ID = 5;
		public static final int LEFT_BACK_STEER_CANCODER_ID = 2;
		public static final int RIGHT_FRONT_STEER_CANCODER_ID = 8;
		public static final int RIGHT_BACK_STEER_CANCODER_ID = 11;

		/* Steer Magnet Offset */
		public static final double LEFT_FRONT_STEER_OFFSET = -200.3;
		public static final double RIGHT_FRONT_STEER_OFFSET = -68.5;
		public static final double LEFT_BACK_STEER_OFFSET = -23.5;
		public static final double RIGHT_BACK_STEER_OFFSET = -208.3;

		/* Drivebase Motor Inversion */
		public static final boolean LEFT_FRONT_SPARK_INVERTED = true;
		public static final boolean LEFT_BACK_SPARK_INVERTED = true;
		public static final boolean RIGHT_FRONT_SPARK_INVERTED = false;
		public static final boolean RIGHT_BACK_SPARK_INVERTED = false;

		/* Drive Train Dimentions */
		public static final double TRACKWIDTH = 0.310;

		/* Motor Encoder Calculations */
		public static final double WHEEL_DIAMTER = 0.1524; // Diameter in meters
		public static final int ENCODER_COUNTS_PER_REV = 4096; // ctre CANCoder
		public static final double DRIVE_GEAR_RATIO = 12.75; // Toughbox mini 12.75:1
		
		public static final double MAX_VELOCITY = 4.4196;
		public static final double MAX_ACCELERATION = 3; // Meters per second
		public static final double MAX_RPS = 183.33; // Max rotations per second

		/* Starting Position */
		public static final double STARTING_X = 0;
		public static final double STARTING_Y = 0;
	}

}