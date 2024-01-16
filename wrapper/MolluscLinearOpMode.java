package org.firstinspires.ftc.teamcode.mollusc.wrapper;

import org.firstinspires.ftc.teamcode.mollusc.Mollusc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class MolluscLinearOpMode extends LinearOpMode {

    @Override
    public final void runOpMode() throws InterruptedException {
        Mollusc.init(this);
        molluscRunOpMode();
        Mollusc.deinit();
    }

    public abstract void molluscRunOpMode() throws InterruptedException;
}

/*
Copyright 2024 Trobotix 8696

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
