package org.firstinspires.ftc.teamcode.mollusc;

import org.firstinspires.ftc.teamcode.mollusc.utility.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Mollusc {

    public static OpMode opMode;

    public static void init(OpMode opMode) {
        opMode = opMode;
        Controls.singlePressMarkers.clear();
        Controls.holdMarkers.clear();
    }

    public static void deinit() {
        opMode = null;
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
