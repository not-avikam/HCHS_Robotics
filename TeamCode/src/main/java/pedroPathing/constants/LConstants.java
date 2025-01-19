package pedroPathing.constants;

import static com.pedropathing.localization.constants.OTOSConstants.angleUnit;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LConstants {
    static {
        OTOSConstants.useCorrectedOTOSClass = false;
        OTOSConstants.hardwareMapName = "I2C1";
        OTOSConstants.linearUnit = DistanceUnit.INCH;
        angleUnit = AngleUnit.RADIANS;
        OTOSConstants.offset = new SparkFunOTOS.Pose2D(-3, 0, 0);
        OTOSConstants.linearScalar = (1.00264470883);
        OTOSConstants.angularScalar = (0.97831226545);
    }
}




