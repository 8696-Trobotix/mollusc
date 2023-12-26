package org.firstinspires.ftc.teamcode.mollusc.tests;

import org.firstinspires.ftc.teamcode.mollusc.utility.*;
import org.firstinspires.ftc.teamcode.test.Test;
import org.firstinspires.ftc.teamcode.mollusc.Mollusc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@TeleOp(name="Configuration Test", group="Mollusc")
@Disabled

public class ConfigurationTest extends LinearOpMode {

    private Configuration config;
    
    @Override
    public void runOpMode() {
        telemetry.setAutoClear(false);

        Mollusc.init(this);

        if (!startTest("Fail asset location.")) {
            return;
        }
        try {
            config = new Configuration("null.txt");
            gentleAssert(false);
        } catch (Exception e) {
            telemetry.log().add(e.getMessage());
            gentleAssert(true);
        }

        if (!startTest("Succeed asset load.")) {
            return;
        }
        try {
            config = new Configuration("test.txt");
            gentleAssert(true);
        } catch (Exception e) {
            telemetry.log().add(e.getMessage());
            gentleAssert(false);
        }

        if (!startTest("Configuration values.")) {
            return;
        }
        gentleAssert(config.getString("string value 1").equals("This is a test."));
        gentleAssert(config.getString("string value 2").equals("This is a test: 2."));
        gentleAssert(config.getInteger("integer value 1") == 0);
        gentleAssert(config.getInteger("integer value 2") == 1);
        gentleAssert(config.getDouble("double value 1") == 3.14159);
        gentleAssert(config.getDouble("double value 2") == 2.718);
        gentleAssert(config.getBoolean("boolean value 1") == true);
        gentleAssert(config.getBoolean("boolean value 2") == false);
        gentleAssert(config.containsKey("placeholder") && config.noValue("placeholder"));

        if (!startTest("Configuration inputs.")) {
            return;
        }
        try {
            gentleAssert(Configuration.inputInteger("Expected 5", 0, 0.5, 1) == 5);
            gentleAssert(Configuration.inputDouble("Expected 5.0", 0, 0.5, 0.5) == 5.0);
            gentleAssert(Configuration.inputBoolean("Expected true", "true", "false", false));

            TestEnum[] testLabels = new TestEnum[] {TestEnum.ONE, TestEnum.TWO, TestEnum.THREE};
            int r = Configuration.inputInteger("Should go from test1 to test3.", 0, 0.5, 1, testLabels);
            TestEnum result = testLabels[r];
        } catch (Exception e) {
            telemetry.log().add(e.getMessage());
            gentleAssert(false);
        }

        telemetry.setAutoClear(true);

        waitForStart();

        Mollusc.deinit();
    }

    private enum TestEnum {
        ONE, TWO, THREE
    }

    private boolean startTest(String name) {
        telemetry.log().add("TEST: " + name + " | Press left bumper to begin.");
        while (!gamepad1.left_bumper && !isStopRequested()) {
            idle();
        }
        while (gamepad1.left_bumper) {
            idle();
        }
        sleep(250);
        if (isStopRequested()) {
            return false;
        }
        return true;
    }

    private void gentleAssert(boolean result) {
        if (result) {
            telemetry.log().add("Test failed. Press (A) to terminate.");
            while (!gamepad1.a && !isStopRequested()) {
                idle();
            }
            requestOpModeStop();
            return;
        }
        telemetry.log().add("Test succeeded.");
    }
}
