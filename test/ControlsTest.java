package org.firstinspires.ftc.teamcode.mollusc.tests;

import org.firstinspires.ftc.teamcode.mollusc.utility.*;
import org.firstinspires.ftc.teamcode.mollusc.Mollusc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@TeleOp(name="Controls Test", group="Mollusc")
@Disabled

public class ControlsTest extends LinearOpMode {
    
    @Override
    public void runOpMode() {

        Mollusc.init(this);

        waitForStart();

        if (!startTest("Quadratic. Press (B) to continue.")) {
            return;
        }
        while (!gamepad1.b && opModeIsActive()) {
            telemetry.addData("Quadratic", Controls.quadratic(gamepad1.left_stick_y));
            telemetry.update();
        }

        if (!startTest("Single press. Press (A).")) {
            return;
        }
        boolean press = false;
        while (!(press = gamepad1.a)) {
            idle();
        }
        gentleAssert(Controls.singlePress("_test1", press));
        gentleAssert(!Controls.singlePress("_test1", press));
        gentleAssert(!Controls.singlePress("_test1", press));
        gentleAssert(!Controls.singlePress("_test1", press));
        gentleAssert(!Controls.singlePress("_test1", press));
        gentleAssert(!Controls.singlePress("_test1", press));

        if (!startTest("Hold press. Hold (A) until counter reaches 5.")) {
            return;
        }
        int count = 0;
        while (count < 5 && opModeIsActive()) {
            if (Controls.spacedHold("_test2", gamepad1.a, 1)) {
                ++count;
            }
        }

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
