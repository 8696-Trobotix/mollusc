package org.firstinspires.ftc.teamcode.mollusc;

import org.firstinspires.ftc.teamcode.mollusc.exception.ParityException;

import org.firstinspires.ftc.teamcode.mollusc.utility.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Mollusc {

    private static OpMode opMode;

    public static void init(OpMode currentOpMode) {
        opMode = currentOpMode;
        Controls.clearMarkers();
    }

    public static void deinit() {
        opMode = null;
    }

    public static LinearOpMode useLinearOpMode() throws ParityException {
        if (!(opMode instanceof LinearOpMode)) {
            throw new ParityException("Telemetry-based configuration is not available with iterative OpModes.");
        }
        return (LinearOpMode)opMode;
    }

    public static OpMode instance() {
        return opMode;
    }
}

/*
Copyright 2023 Trobotix 8696

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
