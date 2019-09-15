package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//Just location and rotation, for Vuforia

public class LocRot {
    public LocRot(VectorF loc, Orientation rot) {
        this.location = loc;
        this.rotation = rot;
    }

    public VectorF location;
    public Orientation rotation;
}