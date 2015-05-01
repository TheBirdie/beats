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

import edu.berkeley.path.beats.simulator.utils.BeatsErrorLog;
import edu.berkeley.path.beats.simulator.utils.BeatsMath;
import edu.berkeley.path.beats.simulator.utils.BeatsTimeProfile;

public final class CapacityProfile extends edu.berkeley.path.beats.jaxb.DownstreamBoundaryCapacityProfile {

	// does not change ....................................
	private Scenario myScenario;
	private boolean isOrphan;
	private double dtinseconds;			// not really necessary
	private int samplesteps;
	private BeatsTimeProfile capacity;	// [veh]
	private int step_initial_abs;       // time steps at start since midnight

	// does change ........................................
	private boolean isdone;
	private double current_sample;

	/////////////////////////////////////////////////////////////////////
	// populate / reset / validate / update
	/////////////////////////////////////////////////////////////////////
	
	protected void populate(Scenario myScenario) {

		this.myScenario = myScenario;
		
		isdone = false;
		
		// required
		Link myLink = myScenario.get.linkWithId(getLinkId());

		isOrphan = myLink==null;
				
		if(isOrphan)
			return;
		
		// set to link
		myLink.myCapacityProfile = this;

		// sample demand distribution, convert to vehicle units
		if(getContent()!=null){
			capacity = new BeatsTimeProfile(getContent(),",");	// true=> reshape to vector along k, define length
			capacity.multiplyscalar(myScenario.get.simdtinseconds()*myLink.get_Lanes());
		}
		
		// optional dt
		if(getDt()!=null){
			dtinseconds = getDt().floatValue();					// assume given in seconds
			samplesteps = BeatsMath.round(dtinseconds / myScenario.get.simdtinseconds());
		}
		else{ 	// allow only if it contains one time step
			if(capacity.getNumTime()==1){
				dtinseconds = Double.POSITIVE_INFINITY;
				samplesteps = Integer.MAX_VALUE;
			}
			else{
				dtinseconds = -1.0;		// this triggers the validation error
				samplesteps = -1;
				return;
			}
		}


        // step_initial
        double start_time = Double.isInfinite(getStartTime()) ? 0d : getStartTime();
        step_initial_abs = BeatsMath.round(start_time/myScenario.get.simdtinseconds());
    }
	
	protected void validate() {

		if(isOrphan){
			BeatsErrorLog.addWarning("Bad origin link ID=" + getLinkId() + " in capacity profile.");
			return;
		}
		
		if(capacity==null || capacity.isEmpty())
			return;

		// check dtinseconds
		if( dtinseconds<=0  && capacity.getNumTime()>1)
			BeatsErrorLog.addError("Non-positive time step in capacity profile for link ID=" + getLinkId());

		if(!BeatsMath.isintegermultipleof(dtinseconds,myScenario.get.simdtinseconds()) && capacity.getNumTime()>1)
			BeatsErrorLog.addError("Time step for capacity profile of link ID=" + getLinkId() + " is not a multiple of simulation time step.");
		
		// check non-negative
		if(capacity.hasNaN())
			BeatsErrorLog.addError("Capacity profile for link ID=" +getLinkId()+ " has illegal values.");

	}

	protected void reset() {			

		if(isOrphan)
			return;
		
		isdone = false;
        current_sample = capacity.get(0);
	}
	
	protected void update(boolean forcesample) {

		if(isOrphan)
			return;

		if(capacity==null || capacity.isEmpty())
			return;
		
		if(isdone)
			return;
		
		if(forcesample || myScenario.get.clock().is_time_to_sample_abs(samplesteps, step_initial_abs)){
			
			int n = capacity.getNumTime()-1;
			int step = myScenario.get.clock().sample_index_abs(samplesteps,step_initial_abs);

			// zeroth sample extends to the left
			if(step<=0){
				current_sample = capacity.get(0);
				return;
			}

			// sample the profile
			if(step<n){
				current_sample = capacity.get(step);
				return;
			}

			// last sample
			if(step>=n && !isdone){
				current_sample = capacity.get(n);
				isdone = true;
				return;
			}
		}
	}

	/////////////////////////////////////////////////////////////////////
	// public interface
	/////////////////////////////////////////////////////////////////////
	
	public double getCurrentValue(){
		return current_sample;
	}

}
