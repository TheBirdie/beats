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

package edu.berkeley.path.beats.actuator;

import edu.berkeley.path.beats.simulator.*;

/**
 * @author Gabriel Gomes (gomes@path.berkeley.edu)
// */
//final public class SignalPhaseX {
//
//	// references ....................................................
//	protected ActuatorSignal mySignal;
////    protected Link[] targetlinks;	// THIS SHOULD BE TARGET INDICES TO THE SIGNAL PHASE CONTROLLER
//
//	// properties ....................................................
//    protected boolean protectd	    = false;
//    protected boolean isthrough	    = false;
//    protected boolean recall		= false;
//    protected boolean permissive	= false;
//    protected boolean lag 		    = false;
//
//	// dual ring structure
//    protected int myRingGroup = -1;
//    protected SignalPhase opposingPhase;
//    protected NEMA.ID myNEMA = NEMA.ID.NULL;
//
//	// Basic timing parameters
//    protected double mingreen;
//    protected double yellowtime;
//    protected double redcleartime;
//    protected double actualyellowtime;
//    protected double actualredcleartime;
//
//	// timers
//    protected Clock bulbtimer;
//
//	// State
//    protected ActuatorSignal.BulbColor bulbcolor;
//
//	//private int [] myControlIndex;
//
//	// Detectors
//	//private DetectorStation ApproachStation = null;
//	//private DetectorStation StoplineStation = null;
//	//private Vector<Integer> ApproachStationIds;
//	//private Vector<Integer> StoplineStationIds;
//
//	// Detector memory
//    protected boolean hasstoplinecall		= false;
//    protected boolean hasapproachcall		= false;
//    protected boolean hasconflictingcall	= false;
//    protected boolean currentconflictcall   = false;
//    protected float conflictingcalltime	= 0f;
//
//	// Safety
//    protected boolean permitopposinghold 	= true;
//    protected boolean permithold			= true;
//
//    // Controller command
//    protected boolean hold_requested 		= false;
//    protected boolean forceoff_requested	= false;
//
//	/////////////////////////////////////////////////////////////////////
//	// construction
//	/////////////////////////////////////////////////////////////////////
//
//	public SignalPhase(Node myNode,ActuatorSignal mySignal,double dt){
//		this.mySignal = mySignal;
//		this.bulbtimer = new Clock(0d,Double.POSITIVE_INFINITY,dt);
//	}
//
//    /////////////////////////////////////////////////////////////////////
//    // actuation command
//    /////////////////////////////////////////////////////////////////////
//
//    protected void setForceoff_requested(boolean forceoff_requested) {
//        this.forceoff_requested = forceoff_requested;
//    }
//
//    protected void setHold_requested(boolean hold_requested) {
//        this.hold_requested = hold_requested;
//    }
//
//	/////////////////////////////////////////////////////////////////////
//	// populate / rese / validate
//	/////////////////////////////////////////////////////////////////////
//
//	protected final void populateFromJaxb(Scenario myScenario,edu.berkeley.path.beats.jaxb.Phase jaxbPhase){
//
//		if(jaxbPhase.getNema()!=null)
//			myNEMA = NEMA.string_to_nema(jaxbPhase.getNema().toString());
//		else
//			myNEMA = NEMA.ID.NULL;
//
//		if(!Double.isNaN(jaxbPhase.getMinGreenTime()))
//			this.mingreen = jaxbPhase.getMinGreenTime();
//		else
//			this.mingreen = Defaults.mingreen;
//
//		if(!Double.isNaN(jaxbPhase.getRedClearTime()))
//			this.redcleartime = jaxbPhase.getRedClearTime();
//		else
//			this.redcleartime = Defaults.redcleartime;
//
//		if(!Double.isNaN(jaxbPhase.getYellowTime()))
//			this.yellowtime = jaxbPhase.getYellowTime();
//		else
//			this.yellowtime = Defaults.yellowtime;
//
//		this.lag = jaxbPhase.isLag();
//		this.permissive = jaxbPhase.isPermissive();
//		this.protectd = jaxbPhase.isProtected();
//		this.recall = jaxbPhase.isRecall();
//
//		// actual yellow and red clear times
//		this.actualyellowtime   = yellowtime;
//		this.actualredcleartime = redcleartime;
//
//		// dual ring structure: opposingPhase, isthrough, myRingGroup
//		switch(myNEMA){
//		case _1:
//			opposingPhase = mySignal.get_phase_with_nema(NEMA.ID._2);
//			isthrough = false;
//			myRingGroup = 0;
//			break;
//		case _2:
//			opposingPhase = mySignal.get_phase_with_nema(NEMA.ID._1);
//			isthrough = true;
//			myRingGroup = 0;
//			break;
//		case _3:
//			opposingPhase = mySignal.get_phase_with_nema(NEMA.ID._4);
//			isthrough = false;
//			myRingGroup = 1;
//			break;
//		case _4:
//			opposingPhase = mySignal.get_phase_with_nema(NEMA.ID._3);
//			isthrough = true;
//			myRingGroup = 1;
//			break;
//		case _5:
//			opposingPhase = mySignal.get_phase_with_nema(NEMA.ID._6);
//			isthrough = false;
//			myRingGroup = 0;
//			break;
//		case _6:
//			opposingPhase = mySignal.get_phase_with_nema(NEMA.ID._5);
//			isthrough = true;
//			myRingGroup = 0;
//			break;
//		case _7:
//			opposingPhase = mySignal.get_phase_with_nema(NEMA.ID._8);
//			isthrough = false;
//			myRingGroup = 1;
//			break;
//		case _8:
//			opposingPhase = mySignal.get_phase_with_nema(NEMA.ID._7);
//			isthrough = true;
//			myRingGroup = 1;
//			break;
//		case NULL:
//			break;
//		default:
//			break;
//		}
//	}
//
//	protected void reset() {
//		hasstoplinecall		= false;
//		hasapproachcall		= false;
//		hasconflictingcall	= false;
//		conflictingcalltime	= 0f;
//		hold_requested 		= false;
//		forceoff_requested	= false;
//		permithold			= true;
//		permitopposinghold  = false;
//        bulbcolor = ActuatorSignal.BulbColor.RED;
//		bulbtimer.reset();
//	}
//
//	protected void validate() {
//
////		// check that there are links attached
////		if(targetlinks==null || targetlinks.length==0)
////			BeatsErrorLog.addError("No valid target link for phase NEMA=" + getNEMA() + " in signal ID=" + signal.getId());
////
////		// target links are valid
////		if(targetlinks!=null)
////			for(int i=0;i<targetlinks.length;i++)
////				if(targetlinks[i]==null)
////					BeatsErrorLog.addError("Unknown link reference in phase NEMA=" + getNEMA() + " in signal ID=" + signal.getId());
//
//		// myNEMA is valid
//		if(myNEMA==NEMA.ID.NULL)
//			BeatsErrorLog.addError("Invalid NEMA code in phase NEMA=" + getNEMA() + " in signal ID=" + mySignal.getId());
//
//		// numbers are positive
//		if( mingreen<0 )
//			BeatsErrorLog.addError("Negative mingreen=" + mingreen + " in signal ID=" + mySignal.getId());
//
//		if( yellowtime<0 )
//			BeatsErrorLog.addError("Negative yellowtime=" + yellowtime + " in signal ID=" + mySignal.getId());
//
//		if( redcleartime<0 )
//			BeatsErrorLog.addError("Negative redcleartime=" + redcleartime + " in signal ID=" + mySignal.getId());
//	}
//
//	/////////////////////////////////////////////////////////////////////
//	// protected
//	/////////////////////////////////////////////////////////////////////
//
//	protected void updatePermitOpposingHold(){
//
//		switch(bulbcolor){
//
//		case GREEN:
//			// iff I am about to go off and there is no transition time
//			permitopposinghold = forceoff_requested && actualyellowtime==0 && redcleartime==0;
//			break;
//		case YELLOW:
//			// iff near end yellow time and there is no red clear time
//			permitopposinghold =  BeatsMath.greaterorequalthan(bulbtimer.getT(),actualyellowtime-bulbtimer.getDt()) && redcleartime==0 ;
//			break;
//		case RED:
//			// iff near end of red clear time and not starting again.
//			permitopposinghold =  BeatsMath.greaterorequalthan(bulbtimer.getT(),redcleartime-bulbtimer.getDt()) && !hold_requested;
//			break;
//		case DARK:
//			break;
//		default:
//			break;
//		}
//	}
//
//	protected ActuatorSignal.BulbColor get_new_bulb_color(boolean hold_approved,boolean forceoff_approved){
//
//        ActuatorSignal.BulbColor next_color = null;
//		double bulbt = bulbtimer.getT();
//
//		if(!protectd)
//            return permissive ? null : ActuatorSignal.BulbColor.RED;
//
//		// execute this state machine until "done". May be more than once if
//		// some state has zero holding time (eg yellowtime=0)
//		boolean done=false;
//
//		while(!done){
//
//			switch(bulbcolor){
//
//			// .............................................................................................
//			case GREEN:
//
////				permitopposinghold = false;
//
//				// Force off
//				if( forceoff_approved ){
//                    next_color = ActuatorSignal.BulbColor.YELLOW;
////					signal.getCompletedPhases().add(signal.new PhaseData(myNEMA, signal.getMyScenario().getClock().getT() - bulbtimer.getT(), bulbtimer.getT()));
//					bulbtimer.reset();
//					//FlushAllStationCallsAndConflicts();
//					done = actualyellowtime>0;
//				}
//				else
//					done = true;
//
//				break;
//
//			// .............................................................................................
//			case YELLOW:
//
//				// set permitopposinghold one step ahead of time so that other phases update correctly next time.
////				permitopposinghold = false;
//
////				if( BeatsMath.greaterorequalthan(bulbt,actualyellowtime-bulbtimer.dt) && redcleartime==0)
////					permitopposinghold = true;
//
//				// yellow time over, go immediately to red if redcleartime==0
//				if( BeatsMath.greaterorequalthan(bulbt,actualyellowtime) ){
//                    next_color = ActuatorSignal.BulbColor.RED;
//					bulbtimer.reset();
//					done = redcleartime>0;
//				}
//				else
//					done = true;
//				break;
//
//			// .............................................................................................
//			case RED:
//
//				//if( BeatsMath.greaterorequalthan(bulbt,redcleartime-myNode.getMyNetwork().getTP()*3600f  && !goG )
////				if( BeatsMath.greaterorequalthan(bulbt,redcleartime-bulbtimer.dt) && !hold_approved )
////					permitopposinghold = true;
////				else
////					permitopposinghold = false;
//
//				// if hold, set to green, go to green, etc.
//				if( hold_approved ){
//                    next_color = ActuatorSignal.BulbColor.GREEN;
//					bulbtimer.reset();
//
//					// Unregister calls (for reading conflicting calls)
//					//FlushAllStationCallsAndConflicts(); // GCG ?????
//
//					done = !forceoff_approved;
//				}
//				else
//					done = true;
//
//				break;
//			case DARK:
//				break;
//			default:
//				break;
//			}
//
//		}
//        return next_color;
//	}
//
//	/////////////////////////////////////////////////////////////////////
//	// protected interface
//	/////////////////////////////////////////////////////////////////////
//
//	public void setActualredcleartime(double actualredcleartime) {
//		if(BeatsMath.lessthan(actualredcleartime,0d))
//			return;
//		this.actualredcleartime = actualredcleartime;
//	}
//
//	public void setActualyellowtime(double actualyellowtime) {
//		if(BeatsMath.lessthan(actualyellowtime,0d))
//			return;
//		this.actualyellowtime = actualyellowtime;
//	}
//
//	protected void setPermithold(boolean permithold) {
//		this.permithold = permithold;
//	}
//
////	public Link[] getTargetlinks() {
////		return targetlinks;
////	}
//
//	protected Clock getBulbtimer() {
//		return bulbtimer;
//	}
//
//	/////////////////////////////////////////////////////////////////////
//	// public interface
//	/////////////////////////////////////////////////////////////////////
//
//    public boolean isGreen(){
//        return bulbcolor.compareTo(ActuatorSignal.BulbColor.GREEN)==0;
//    }
//
//    public boolean isYellow(){
//        return bulbcolor.compareTo(ActuatorSignal.BulbColor.YELLOW)==0;
//    }
//
//    public boolean isRed(){
//        return bulbcolor.compareTo(ActuatorSignal.BulbColor.RED)==0;
//    }
//
//    public boolean isProtected() {
//		return protectd;
//	}
//
//	public boolean isPermitopposinghold() {
//		return permitopposinghold;
//	}
//
//	public boolean isHold_requested() {
//		return hold_requested;
//	}
//
//	public boolean isForceoff_requested() {
//		return forceoff_requested;
//	}
//
//	public boolean isPermithold() {
//		return permithold;
//	}
//
//	public boolean isIsthrough() {
//		return isthrough;
//	}
//
//	public boolean isPermissive() {
//		return permissive;
//	}
//
//	public SignalPhase getOpposingPhase() {
//		return opposingPhase;
//	}
//
//	public double getYellowtime() {
//		return yellowtime;
//	}
//
//	public double getRedcleartime() {
//		return redcleartime;
//	}
//
//	public double getMingreen() {
//		return mingreen;
//	}
//
//	public NEMA.ID getNEMA() {
//		return myNEMA;
//	}
//
//	public double getActualyellowtime() {
//		return actualyellowtime;
//	}
//
//	public double getActualredcleartime() {
//		return actualredcleartime;
//	}
//
//	public ActuatorSignal.BulbColor getBulbColor() {
//		return bulbcolor;
//	}
//
//}
