package org.firstinspires.ftc.teamcode.mollusc.vision;

public class VisionObject {

    // Units are in pixels.
    // The origin (0, 0) is located in the upper left most corner.
    public int x, y, width, height;
    // Totality is a ratio of the number of pixels assumed to be a part of the object to the total number of pixels.
    public double pixelTotality, boundingTotality;

    public VisionObject(int x, int y, int width, int height, double pixelTotality, double boundingTotality) {
        this.x = x;
        this.y = y;
        this.width = width;
        this.height = height;
        this.pixelTotality = pixelTotality;
        this.boundingTotality = boundingTotality;
    }

    @Override
    public String toString() {
        return String.format(
            "x = %d, y = %d, width = %d, height = %d, pixel totality = %.3f, bounding totality = %.3f", 
            x, y, width, height, pixelTotality, boundingTotality
        );
    }
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
