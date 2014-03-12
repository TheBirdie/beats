///**
// * Copyright (c) 2012, Regents of the University of California
// * All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without
// * modification, are permitted provided that the following conditions are met:
// *
// *   Redistributions of source code must retain the above copyright notice,
// *   this list of conditions and the following disclaimer.
// *   Redistributions in binary form must reproduce the above copyright notice,
// *   this list of conditions and the following disclaimer in the documentation
// *   and/or other materials provided with the distribution.
// *
// * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// * POSSIBILITY OF SUCH DAMAGE.
// **/

//package edu.berkeley.path.beats.control;
//
//import java.util.ArrayList;
//
//import edu.berkeley.path.beats.actuator.ActuatorSignal;
//import edu.berkeley.path.beats.simulator.Scenario;
//
//public class Controller_SIG_Pretimed_Plan { //} extends Controller_SIG_Pretimed.Plan {
//
//	protected Controller_SIG_Pretimed myController;
//
//	// input parameters
//	protected Controller_SIG_Pretimed_IntersectionPlan [] intersplan;
//	protected boolean [] havesignaltarget;	// true if this intersection is in the target list
//	protected double _cyclelength;
//
//	ArrayList<SignalCommand> commandlist = new ArrayList<SignalCommand>();

	/////////////////////////////////////////////////////////////////////
	// populate / validate / reset  / update
	/////////////////////////////////////////////////////////////////////
	
//	protected void populate(Controller_SIG_Pretimed myController, Scenario myScenario, Controller_SIG_Pretimed.Plan plan) {
		
//		this.myController = myController;
//		
//		this.setId(plan.getId());
//		
//		if (null != plan.getCycleLength())
//			_cyclelength = plan.getCycleLength().doubleValue();
//					
//		if (null != plan.getIntersection()) {
//			int numintersection = plan.getIntersection().size();
//			havesignaltarget = new boolean[numintersection];
//			intersplan = new Controller_SIG_Pretimed_IntersectionPlan[numintersection];
//			for(int i=0;i<intersplan.length;i++){
//
//				// check whether the signal is in the target list
//				Controller_SIG_Pretimed.Intersection intersection = plan.getIntersection().get(i);
//				ActuatorSignal signal = myScenario.getSignalWithNodeId(intersection.getNodeId());
//				if(signal==null)
//					continue;
//				boolean haveit = false;
//				for(Actuator act : myController.actuators){
//					if( se.getMyType().compareTo(ScenarioElement.Type.signal)==0 && se.getId()==signal.getId() ){
//						haveit=true;
//					}
//				}
//				if(!haveit)
//					continue;				
//				intersplan[i] = new Controller_SIG_Pretimed_IntersectionPlan(this);
//				intersplan[i].populate(myScenario, intersection);
//			}
//		}
		
//	}
//
//	protected void validate(){
//
//		if(myController==null)
//			BeatsErrorLog.addError("Invalid controller for pretimed signal plan ID=" + getId() + ".");
//		
//		// positive cycle
//		if(_cyclelength<=0)
//			BeatsErrorLog.addError("Non-positive cycle length in pretimed signal controller ID=" + getId() + ".");
//		
//		// cycle length should be a multiple of controller dt
//		if(myController!=null)
//			if(!BeatsMath.isintegermultipleof(_cyclelength,myController.getDtinseconds()))
//				BeatsErrorLog.addError("Cycle length is not an integer multiple of controller rate in pretimed signal controller ID=" + getId()+ ".");
//		
//		// plan includes all targets
//		boolean foundit;
//		if(myController!=null)
//			for(ScenarioElement se : myController.getTargets()){
//				foundit = false;
//				for(int i=0;i<intersplan.length;i++){
//					if(se.getId()==intersplan[i].signal.getId()){
//						foundit=true;
//						break;
//					}
//				}
//				if(!foundit)
//					BeatsErrorLog.addError("Controller target (ID="+se.getId()+") not found in pretimed signal plan ID="+getId());
//			}
//		
//		// intersection plans
//		for(int i=0;i<intersplan.length;i++)
//			intersplan[i].validate(myController.getDtinseconds());
//
//	}

//	protected void reset() {
//		for(int i=0;i<intersplan.length;i++)
//			intersplan[i].reset();
//	}
//
//	/////////////////////////////////////////////////////////////////////
//	// protected methods
//	/////////////////////////////////////////////////////////////////////
//
//	protected void implementPlan(double simtime,boolean coordmode){
//
//		int i;
//		double itime;
//
//		// Master clock .............................
//		itime =  simtime % _cyclelength;
//
//		// Loop through intersections ...............
//		for(i=0;i<intersplan.length;i++){

//			commandlist.clear();
//
//			// get commands for this intersection
//			intersplan[i].getCommandForTime(itime,commandlist);
//
//			// send command to the signal
//			intersplan[i].signal.set_command(commandlist);

//			if( !coordmode ){
//				for(j=0;j<intplan.holdpoint.length;j++)
//					if( reltime==intplan.holdpoint[j] )
//						intplan.signal.IssueHold(j);
//
//				for(j=0;j<intplan.holdpoint.length;j++)
//					if( reltime==intplan.forceoffpoint[j] )
//						intplan.signal.IssueForceOff(j,intplan.signal.phase[j].actualyellowtime,intplan.signal.phase[j].actualredcleartime);
//			}

			
			// Used for coordinated actuated.
//			if( coordmode ){

//			for( j=0;j<8;j++ ){
//
//					
//					if( !intplan.signal.Phase(j).Protected() )
//						continue;
//
//					issyncphase = j==intplan.movA[0] || j==intplan.movB[0];
//
//					// Non-persisting forceoff request at forceoffpoint
//					if( reltime==intplan.forceoffpoint[j] )
//						c.setRequestforceoff(i, j, true);
//
//					// Hold request for sync phase if
//					// currently both sync phases are active
//					// and not reached syncpoint
//					if( issyncphase && 
//						c.PhaseA(i)!=null && c.PhaseA(i).MyNEMA()==intplan.movA.get(0) && 
//						c.PhaseB(i)!=null && c.PhaseB(i).MyNEMA() == intplan.movB.get(0) &&
//						reltime!= c.Syncpoint(i) )
//						c.setRequesthold(i, j, true);
//				}
//			}
//		}
//	}
//
//}
