package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = 0.0029;
        ThreeWheelConstants.strafeTicksToInches = .0029;
        ThreeWheelConstants.turnTicksToInches = 0.0029;
        ThreeWheelConstants.leftY = 6;
        ThreeWheelConstants.rightY = -6;
        ThreeWheelConstants.strafeX = 5;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "bl";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "fr";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "fl";
        ThreeWheelConstants.leftEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.rightEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.REVERSE;
    }
}




