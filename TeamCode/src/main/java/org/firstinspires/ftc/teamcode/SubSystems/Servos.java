package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.teamcode.Constants.Hardware.armServo;
import static org.firstinspires.ftc.teamcode.Constants.Hardware.clawServo;
import static org.firstinspires.ftc.teamcode.Constants.Hardware.wristServo;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Constants.Hardware;
import org.firstinspires.ftc.teamcode.Constants.RobotConstants;

public class Servos {

    public enum ClawGrabState {
        CLOSED,
        OPEN
    }

    public ClawGrabState grabState;

    public void setGrabState(ClawGrabState clawGrabState) {
        if (clawGrabState == ClawGrabState.CLOSED) {
            clawServo.setPosition(RobotConstants.clawClose);
            this.grabState = ClawGrabState.CLOSED;
        } else if (clawGrabState == ClawGrabState.OPEN) {
            clawServo.setPosition(RobotConstants.clawOpen);
            this.grabState = ClawGrabState.OPEN;
        }
    }

    public void switchGrabState() {
        if (grabState == ClawGrabState.CLOSED) {
            setGrabState(ClawGrabState.OPEN);
        } else if (grabState == ClawGrabState.OPEN) {
            setGrabState(ClawGrabState.CLOSED);
        }
    }

    public void openClaw() {
        setGrabState(ClawGrabState.OPEN);
    }

    public void closeClaw() {
        setGrabState(ClawGrabState.CLOSED);
    }

    public enum WristPivotState {
        UP,
        MIDDLE,
        DOWN
    }

    public WristPivotState pivotState;

    public void setPivotState(WristPivotState wristPivotState) {
        if (wristPivotState == WristPivotState.UP) {
            wristServo.setPosition(RobotConstants.wristUp);
            this.pivotState = WristPivotState.UP;
        } else if (wristPivotState == WristPivotState.MIDDLE) {
            wristServo.setPosition(RobotConstants.wristMiddle);
            this.pivotState = WristPivotState.MIDDLE;
        } else if (wristPivotState == WristPivotState.DOWN) {
            wristServo.setPosition(RobotConstants.wristDown);
            this.pivotState = WristPivotState.DOWN;
        }
    }

    public void switchPivotState() {
        if (pivotState == WristPivotState.UP) {
            setPivotState(WristPivotState.DOWN);
        } else if (pivotState == WristPivotState.DOWN) {
            setPivotState(WristPivotState.UP);
        }
    }

    public void wristDown() {
        setPivotState(WristPivotState.DOWN);
    }

    public void wristMiddle() {
        setPivotState(WristPivotState.MIDDLE);
    }

    public void wristUp() {
        setPivotState(WristPivotState.UP);
    }

    public enum ArmState {
        OUT,
        MIDDLE,
        IN
    }

    public ArmState armState;

    public void setArmState(ArmState state) {
        if (state == ArmState.OUT) {
            armServo.setPosition(RobotConstants.armOut);
            this.armState = ArmState.OUT;
        } else if (state == ArmState.MIDDLE) {
            armServo.setPosition(RobotConstants.armMiddle);
            this.armState = ArmState.MIDDLE;
        } else if (state == ArmState.IN) {
            armServo.setPosition(RobotConstants.armIn);
            this.armState = ArmState.IN;
        }
    }

    public void switchArmState() {
        if (armState == ArmState.IN) {
            setArmState(ArmState.OUT);
        } else if (armState == ArmState.OUT) {
            setArmState(ArmState.IN);
        }
    }

    public void armOut() {
        setArmState(ArmState.OUT);
    }

    public void armMiddle() {
        setArmState(ArmState.MIDDLE);
    }

    public void armIn() {
        setArmState(ArmState.IN);
    }
}
