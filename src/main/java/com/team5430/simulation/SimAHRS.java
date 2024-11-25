package com.team5430.simulation;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.SPI.Port;

public class SimAHRS extends AHRS{

  
// TODO: fix, not updating properly; not worked on enough yet
    protected int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");

    public SimBoolean isConnected = new SimBoolean(SimDeviceDataJNI.getSimValueHandle(dev, "Connected"));
    public SimDouble rate = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Rate"));
    public SimDouble yaw = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    public SimDouble pitch = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Pitch"));
    public SimDouble roll = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Roll"));
    public SimDouble fusedHeading = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "FusedHeading"));
    public SimDouble CompassHeading = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "CompassHeading"));
    public SimDouble LinearWorldAccelX = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "LinearWorldAccelX"));
    public SimDouble LinearWorldAccelY = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "LinearWorldAccelY"));
    public SimDouble LinearWorldAccelZ = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "LinearWorldAccelZ"));


    public SimAHRS(){

        super();
      
    }

    public SimAHRS(Port kmxp) {
        super(kmxp);
    }

    public void initSim(){
         isConnected.set(true);
        yaw.set(0);
        CompassHeading.set(0);
        LinearWorldAccelX.set(0);
        fusedHeading.set(0);
        LinearWorldAccelY.set(0);
        roll.set(0);
        pitch.set(0);
    }


}

