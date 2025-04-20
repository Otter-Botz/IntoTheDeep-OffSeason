package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = 0.072625;
        ThreeWheelConstants.strafeTicksToInches = 0.00525;
        ThreeWheelConstants.turnTicksToInches = 0.006125;
        ThreeWheelConstants.leftY = 1;
        ThreeWheelConstants.rightY = -1;
        ThreeWheelConstants.strafeX = -2.5;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "frontLeft";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "backRight";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "backLeft";
        ThreeWheelConstants.leftEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.rightEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.REVERSE;

    }
}




