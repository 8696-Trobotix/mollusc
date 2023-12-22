package org.firstinspires.ftc.teamcode.mollusc.tests;

import org.firstinspires.ftc.teamcode.mollusc.utility.*;
import org.firstinspires.ftc.teamcode.mollusc.Mollusc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@TeleOp(name="Asset Test", group="Mollusc")
@Disabled

public class AssetTest extends LinearOpMode {
    
    @Override
    public void runOpMode() {

        Mollusc.init(this);

        if (!startTest("Succeed asset load.")) {
            return;
        }

        Asset asset = null;
        try {
            asset = new Asset("test.txt");
            gentleAssert(true);
        } catch (Exception e) {
            telemetry.log().add(e.getMessage());
            gentleAssert(false);
        }
        gentleAssert(asset != null);

        if (!startTest("Raw data.")) {
            return;
        }
        gentleAssert(asset.getData().equals("// Test Configuration File\n\n// This file must be located in an assets folder for use in testing.\n\nstring value 1: This is a test.\nstring value 2: This is a test: 2.\n\ninteger value 1: 0\ninteger value 2: 1\n\ndouble value 1: 3.14159\ndouble value 2: 2.718\n\nboolean value 1: true\nboolean value 2: false\n\nplaceholder\n"));

        if (!startTest("Lines.")) {
            return;
        }
        String[] expectedLines = new String[] {
            "// Test Configuration File", 
            "", 
            "// This file must be located in an assets folder for use in testing.", 
            "", 
            "string value 1: This is a test.", 
            "string value 2: This is a test: 2.", 
            "", 
            "integer value 1: 0", 
            "integer value 2: 1", 
            "", 
            "double value 1: 3.14159", 
            "double value 2: 2.718", 
            "", 
            "boolean value 1: true", 
            "boolean value 2: false", 
            "", 
            "placeholder", 
            ""
        };
        String[] lines = asset.getLines();
        boolean linesSuccess = true;
        if (expectedLines.length != lines.length) {
            gentleAssert(false);
        } else {
            for (int i = 0; i < lines.length; ++i) {
                if (!expectedLines[i].equals(lines[i])) {
                    linesSuccess = false;
                    break;
                }
            }
        }
        gentleAssert(linesSuccess);

        if (!startTest("Tokens.")) {
            return;
        }
        String[][] expectedTokens = new String[][] {
            new String[] {"//", "Test", "Configuration", "File"}, 
            new String[] {}, 
            new String[] {"//", "This", "file", "must", "be", "located", "in", "an", "assets", "folder", "for", "use", "in", "testing."}, 
            new String[] {}, 
            new String[] {"string", "value", "1:", "This", "is", "a", "test."}, 
            new String[] {"string", "value", "2:", "This", "is", "a", "test:", "2."}, 
            new String[] {}, 
            new String[] {"integer", "value", "1:", "0"}, 
            new String[] {"integer", "value", "2:", "1"}, 
            new String[] {}, 
            new String[] {"double", "value", "1:", "3.14159"}, 
            new String[] {"double", "value", "2:", "2.718"}, 
            new String[] {}, 
            new String[] {"boolean", "value", "1:", "true"}, 
            new String[] {"boolean", "value", "2:", "false"}, 
            new String[] {}, 
            new String[] {"placeholder"}
        };
        String[][] tokens = asset.getTokens();
        boolean tokensSuccess = true;
        if (expectedTokens.length != tokens.length) {
            gentleAssert(false);
        } else {
            for (int i = 0; i < tokens.length; ++i) {
                if (expectedTokens[i].length != tokens[i].length) {
                    tokensSuccess = false;
                    break;
                }
            }
        }
        gentleAssert(tokensSuccess);

        waitForStart();

        Mollusc.deinit();
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
