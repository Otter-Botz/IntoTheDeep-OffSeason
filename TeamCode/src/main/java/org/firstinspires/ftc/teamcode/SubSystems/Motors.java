package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.teamcode.Constants.Hardware.sliderMotor;
import static org.firstinspires.ftc.teamcode.Constants.Hardware.sliderMotorMotor;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants.RobotConstants;

public class Motors {

    public void controlSlider(double power, int targetPosition) {
        sliderMotor.setTargetPosition(targetPosition);
        sliderMotorMotor.setTargetPosition(targetPosition);

        sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderMotorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sliderMotor.setPower(power);
        sliderMotorMotor.setPower(power);

        while (RobotConstants.isOpModeActive() && sliderMotorMotor.isBusy() && sliderMotor.isBusy()) {
            // Wait
        }
        RobotConstants.isSliderHold = false;

        sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderMotorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
