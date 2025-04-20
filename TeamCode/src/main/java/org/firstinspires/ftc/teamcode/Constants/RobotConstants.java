package org.firstinspires.ftc.teamcode.Constants;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {
    public static double clawClose = 0;
    public static double clawOpen = 1;

    public static double wristUp = 1;
    public static double wristMiddle = 0.5;
    public static double wristDown = 0;

    public static double armOut = 1;
    public static double armMiddle = 0.5;
    public static double armIn = 0;

    public static boolean isSliderHold = false;

    public static int [] sliderPositions = {
            0,
            100,
            200,
            300,
            400,
            500,
            600,
            700,
            800,
            900,
            1000
    };
    public static double [] sliderPower = {
            0,
            0.1,
            0.2,
            0.3,
            0.4,
            0.5,
            0.6,
            0.7,
            0.8,
            0.9,
            1
    };

    public static boolean isOpModeActive(){
        return linearOpMode.opModeIsActive();
    }
}
