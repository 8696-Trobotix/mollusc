package org.firstinspires.ftc.teamcode.mollusc.tests;

import org.firstinspires.ftc.teamcode.mollusc.utility.*;
import org.firstinspires.ftc.teamcode.mollusc.exception.*;
import org.firstinspires.ftc.teamcode.mollusc.Mollusc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@TeleOp(name="Parity Test", group="Mollusc")
@Disabled

public class ParityTest extends OpMode {

    private boolean success = true;

    /*
    * Code to run ONCE when the driver hits INIT
    */
    @Override
    public void init() {
        Mollusc.init(this);        
    }

    /*
    * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    */
    @Override
    public void init_loop() {
        if (success) {
            try {
                // This should fail and throw an exception.
                Configuration.inputBoolean("Placeholder", "1", "0", false);
            } catch (Exception e) {
                telemetry.log().add(e.getMessage());
                success = false;
            }
        } else {
            telemetry.log().add("Test succeeded. Proceed to terminate OpMode.");
        }
    }

    /*
    * Code to run ONCE when the driver hits PLAY
    */
    @Override
    public void start() {
    }

    /*
    * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    */
    @Override
    public void loop() {
    }

    /*
    * Code to run ONCE after the driver hits STOP
    */
    @Override
    public void stop() {
    }

}
