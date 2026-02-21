package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.util.underglow.Color;
import org.firstinspires.ftc.teamcode.util.underglow.PrismAnimations;

public class robotConstants {
    @Config()
    public static class FieldConstants{

        public static Pose blueGoalFieldPose = new Pose(130.0, 132.0);
        public static Pose redGoalFieldPose = new Pose(130.0, 135.0);

        public static Pose blueAllianceFarSideStartingPose = new Pose(10.0, 88.0);

        public static Pose robotStartingPose = new Pose(72.0, 72.0);
    }





    @Config
    public static class RobotConstants{
        public static enum Alliance {BlueAlliance, RedAlliance}
        public static Alliance selectedAlliance = Alliance.BlueAlliance;


        public static enum DriveMode {RobotCentric, FieldCentric}
        public static DriveMode selectedDriveMode = DriveMode.FieldCentric;


        public static String frontLeftMotorName="FL";
        public static String backLeftMotorName="BL";
        public static String frontRightMotorName="FR";
        public static String backRightMotorName="BR";


        public static boolean frontLeftMotorInverted=false;
        public static boolean backLeftMotorInverted=false;
        public static boolean frontRightMotorInverted=true;
        public static boolean backRightMotorInverted=true;


        public static boolean floatModeEnabled=true;


        public static double driveCubicTerm=0.5, driveLinearTerm=0.4, minimumDriveTrainSpeed=0.415;
        public static double headingKp=0, headingKd=0;
    }





    @Config
    public static class UnderglowConstants{
        public static String goBildaPrismDriverName="prismDriver";

        public static PrismAnimations.Solid underGlowColor = new PrismAnimations.Solid(Color.RED);
    }





    @Config
    public static class SpindexerConstants{
        public static String leftSpindexerServoName="leftSpindexerServo";
        public static String rightSpindexerServoName="rightSpindexerServo";
        public static String encoderName="intakeMotor";

        public static double spindexerIntakingPositions [] = {0.039, 0.135, 0.23};
        public static double spindexerTransferingPositions [] = {0.0845, 0.18, 0.278};

        public static int spindexerVelocityThreshold=2200;
    }





    @Config
    public static class DistanceSensorConstants{
        public static String leftDistanceSensorName="leftDistanceSensor";
        public static String rightDistanceSensorName="rightDistanceSensor";

        public static double slotOccupiedThreshold=15.80;
        public static double spindexerVelocityZeroedTolerance=15000;
        public static double spindexerIndexDelay=250;
    }





    @Config
    public static class TransferConstants{
        public static String transferServoName="TransferServo";

        public static double transferPosition=0.7;
        public static double transferHomePosition=0.4;

        public static long transferServoRiseTime=135;
        public static long transferHomingTime=350;
    }





    @Config
    public static class GamePadControls{
        public static GamepadEx gamepad1EX;




        public static GamepadKeys.Button switchDriveModeMapping = GamepadKeys.Button.DPAD_UP;

        public static GamepadKeys.Button enableIntakeMapping = GamepadKeys.Button.RIGHT_STICK_BUTTON;
        public static GamepadKeys.Button reverseIntakeMapping = GamepadKeys.Button.LEFT_STICK_BUTTON;

        public static GamepadKeys.Button transferArtifactsMapping = GamepadKeys.Button.A;





        public static GamepadKeys.Button offsetTurretLeftMapping = GamepadKeys.Button.LEFT_BUMPER;
        public static GamepadKeys.Button offsetTurretRightMapping = GamepadKeys.Button.RIGHT_BUMPER;





        public static GamepadKeys.Button disableTurretMapping = GamepadKeys.Button.Y;
    }




    @Config
    public static class ShooterConstants{
        public enum ShooterState {Accelerating, TargetReached}
        public static ShooterState currentShooterState = ShooterState.Accelerating;

        public static String indicatorLightServoName="indicatorLight";
        public static String pitcherServoName="pitcherServo";
        public static String leftFlyWheelMotorName="leftFlyWheelMotor";
        public static String rightFlyWheelMotorName="rightFlyWheelMotor";

        public static boolean leftFlyWheelMotorInverted=true;
        public static boolean rightFlyWheelMotorInverted=false;

        public static boolean floatModeEnabled=false;


        public static double flyWheelKp=0.005, flyWheelKi=0.0, flyWheelKd=0.0, flyWheelKf= 0.00038, flyWheelKs=0.089;
        public static double nominalVoltage=13.4, maxVelocityAtNominalVoltage=3000.0, velocityTolerance=40.0;



        public static double indicatorLightAcceleratingPosition=0.3;
        public static double indicatorLightTargetReachedPositon=0.5;
    }





    @Config
    public static class TurretConstants{
        public static String turretMotorName="turretMotor";
        public static boolean turretMotorInverted=true;
        public static boolean floatModeEnabled=false;

        public static double turretKp=0.007, turretKd=0.0;
        public static double turretMaxPower=0.6;


        public static double turretLeftMaxAngle=0;
        public static double turretRightMaxAngle=240;

        public static double trackingOffsset=10.25;

    }





    @Config
    public static class IntakeConstants{
        public enum IntakeState {Off, Intaking, Reversing}
        public static IntakeState currentIntakeState = IntakeState.Off;

        public static String intakeMotorName="intakeMotor";
        public static boolean intakeMotorInverted=false;
        public static boolean floatModeEnabled=false;

        public static double intakingSpeed=1.0;
        public static double reversingSpeed=-1.0;
        public static double disabledSpeed=0.0;
    }
}