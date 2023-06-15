// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double TAU = Math.PI*2;
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class Arm {
    public static final double PIVOT_GEARRATIO = 4*4*4*72/22.; //4^3*72 rotations of the motor = 22 rotation of the sprocket
    public static final double NU_PER_MM = 58.4;
    public static final double MM_PER_NU = 0.0171;

    public static final double l0 = 0.690;  //Initial length of arm in meters

    public static final int PIVOT_PORT_1 = 13;
    public static final int PIVOT_PORT_2 = 15;

    public static final int ARM_PORT = 10;

    public static final int EXTEND_SWITCH_PORT = 0;
    public static final int RETRACT_SWITCH_PORT = 0;

    public static final int ENCODER_PORT = 4;
    public static final double ENCODER_OFFSET = -0.192383;

    public static final double ARM_KF = 0.016;
    public static final double ARM_KP = 2.2;
    public static final double ARM_KI = 0;
    public static final double ARM_KD = 0;

    public static final double ARM_CRUISE_VELOCITY = 100;  //40_000
    public static final double AUTO_ARM_CRUISE_VELOCITY = 100;
    public static final double ARM_ACCELERATION = 160;
    public static final double AUTO_ARM_ACCELERATION = 100;

    public static final double PIVOT_CRUISE_VELOCITY = 100;
    public static final double AUTO_PIVOT_CRUISE_VELOCITY = 100;
    public static final double PIVOT_ACCELERATION = 300;
    public static final double AUTO_PIVOT_ACCELERATION = 120;

    public static final double PIVOT_KF = 0.0095;
    public static final double PIVOT_KP = 0.5;
    public static final double PIVOT_KI = 0;
    public static final double PIVOT_KD = 0.015;

    //Scorign (Radians)
    public static final double HIGH_FRONT_ANGLE = 63.75 * TAU/360;
    public static final double HIGH_BACK_ANGLE = -61.5 * TAU/360;
    public static final double MID_ANGLE = 70 * TAU/360;
    public static final double LOW_ANGLE = 104 * TAU/360;

    public static final double HIGH_EXTEND_NU = 80_000/2048.; //47_000-2_600
    public static final double MID_EXTEND_NU = 29_500/2048.;
    public static final double LOW_EXTEND_NU = 0/2048.;

    public static final double AUTO_DUNK_ANGLE = 3 * TAU/360;
    public static final double TELEOP_DUNK_ANGLE = 1 * TAU/360;

    public static final double RETRACT_NU = 18_000/2048.; //NU to retract back in after scoring to avoid hitting middle node

    public static final double PREP_ANGLE = 22*TAU/360;
    public static final double EXTEND_REST_NU = 3;

    //Intkae
    public static final double INTAKE_ANGLE = 116 * Constants.TAU/360; //112.5
    public static final double INTAKE_EXTEND_NU = 3_000/2048.;
    public static final double INTAKE_DEADZONE = 1 * TAU/360;

    public static final double SINGLE_STATION_ANGLE = 76 * TAU/360;
    public static final double SINGLE_STATION_EXTEND_NU = 0;

    public static final double ERROR_ANGLE = 0;

    public static final int EXTEND_FORWARD_SOFT_LIMIT = 48;
    public static final double EXTEND_REVERSE_SOFT_LIMIT = EXTEND_REST_NU;

    public static final double PIVOT_FORWARD_SOFT_LIMIT = 140_000/2048.;
    public static final double PIVOT_REVERSE_SOFT_LIMIT = -140_000/2048.;

    public static final double EXTEND_TARGET_DEADZONE = 1;
    public static final double PIVOT_TARGET_DEADZONE = 0.5 * TAU / 360;
    
    public static final class Cube{
      public static final double HIGH_ANGLE = -59 * TAU/360;
      public static final double MID_ANGLE = -62 * TAU/360;
      public static final double LOW_ANGLE = -105 * TAU/360;

      public static final double INTAKE_ANGLE = -108 * Constants.TAU/360;
      public static final double INTAKE_EXTEND_NU = 3_000/2048.;

      public static final double HIGH_EXTEND_NU = 82_000/2048.;
      public static final double MID_EXTEND_NU = 35_000/2048.;
      public static final double LOW_EXTEND_NU = 0/2048.;
    }
  }
}
