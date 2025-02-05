/*
Copyright (c) 2024 Limelight Vision

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class LimeLight {

    public Limelight3A limelight;
    DriveBase Drive = new DriveBase();


    private double tx, ty, ta;

    public enum limelightState {
        yellow,
        red,
        blue,
        aprilTag,
        none
    }

    private Telemetry telemetry;


    public limelightState state;
    private LLResult result;

    private int pipeline = 0;
    private double x = 0;
    private double y = 0;

    public static double extendMultipler = 0.0045;


    public void init(HardwareMap hwMap) {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        Drive.init(hardwareMap);
    }

    public void start() {
        limelight.start();
        limelight.pipelineSwitch(pipeline);
    }

    public void switchPipeline(limelightState state) {
        switch (state) {
            case yellow:
                pipeline = 0;
                break;
            case red:
                pipeline = 1;
                break;
            case blue:
                pipeline = 2;
                break;
            case aprilTag:
                pipeline = 3;
                break;
        }

        limelight.pipelineSwitch(pipeline);

        if (state == limelightState.none) {
            limelight.stop();
        }
    }




    public void updateColor() {
        update();
        if (result != null) {
            telemetry.addData("tx", result.getTx());
            telemetry.addData("ty", result.getTy());
        }
    }

    public void extendAlign(double error) {
//        if(error > -25) {
//            extend.setLimitToSample();
//            double extendDistance = -(-25-error) * extendMultipler;
//
//            if(extend.getPos() + extendDistance > extend.extendLimit)
//                extend.setTarget(extend.extendLimit);
//            else
//                extend.setTarget(extend.getPos() + extendDistance);
//
//
//        }


    }

    public void driveAlign(double error) {


    }

    public void update() {
        result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            tx = result.getTx(); // How far left or right the target is (degrees)
            ty = result.getTy(); // How far up or down the target is (degrees)
            ta = result.getTa(); // How big the target looks (0%-100% of the image)

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);
        } else {
            telemetry.addData("Limelight", "No Targets");
        }

        telemetry.update();
    }

    public double getTxError() {
        update();
        return result.getTx();
    }

    public double getTyError() {
        update();
        return result.getTy();
    }

    public void strafeLeft(double left) {
        double leftFrontPower = -left;
        double rightFrontPower = left;
        double leftBackPower = left;
        double rightBackPower = -left;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        Drive.BackLeftMotor.setPower(leftFrontPower);
        Drive.BackRightMotor.setPower(rightFrontPower);
        Drive.FrontLeftMotor.setPower(leftBackPower);
        Drive.FrontRightMotor.setPower(rightBackPower);
    }




}
