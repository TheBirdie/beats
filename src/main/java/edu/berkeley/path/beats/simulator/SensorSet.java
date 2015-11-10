/**
 * Copyright (c) 2012, Regents of the University of California
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *   Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *   Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 **/

package edu.berkeley.path.beats.simulator;

import java.io.Serializable;
import java.util.ArrayList;

import edu.berkeley.path.beats.jaxb.SensorType;
import edu.berkeley.path.beats.simulator.utils.BeatsException;

public final class SensorSet extends edu.berkeley.path.beats.jaxb.SensorSet implements Serializable {

	private static final long serialVersionUID = -8240089073029445302L;

	private ArrayList<Sensor> sensors = new ArrayList<Sensor>();

	/////////////////////////////////////////////////////////////////////
	// protected interface
	/////////////////////////////////////////////////////////////////////

	protected ArrayList<Sensor> getSensors() {
		return sensors;
	}
	
	/////////////////////////////////////////////////////////////////////
	// populate / reset / validate / update
	/////////////////////////////////////////////////////////////////////
	
	protected void populate(Scenario myScenario) {
		
		// replace jaxb.Sensor with simulator.Sensor
		if(myScenario.getSensorSet()!=null){
			for(edu.berkeley.path.beats.jaxb.Sensor sensorjaxb : myScenario.getSensorSet().getSensor()) {

				// assign type
				Sensor.Type myType = null;
		    	try {
		    		SensorType stype = sensorjaxb.getSensorType();
		    		if(stype!=null)
		    			myType = Sensor.Type.valueOf(stype.getName().toLowerCase());
				} catch (IllegalArgumentException e) {
					continue;
				}
				
				// generate sensor
				if(myType!=null){
					Sensor S = ObjectFactory.createSensorFromJaxb(myScenario,sensorjaxb,myType);
					if(S!=null)
						sensors.add(S);
				}		    	
			}
		}
	}

	protected void validate() {
		for(Sensor sensor : sensors)
			sensor.validate();
	}
	
	public void reset() throws BeatsException {
		for(Sensor sensor : sensors)
			sensor.reset();
	}

    public void update() throws BeatsException {

        // NOTE: ensembles have not been implemented for sensors. They do not apply
        // to the loop sensor, but would make a difference for floating sensors.
		for(Sensor sensor : sensors)
			sensor.update();
	}

}
