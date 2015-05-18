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

package edu.berkeley.path.beats.control;

import edu.berkeley.path.beats.simulator.Controller;
import edu.berkeley.path.beats.simulator.Link;
import edu.berkeley.path.beats.simulator.Scenario;
import edu.berkeley.path.beats.simulator.Sensor;
import edu.berkeley.path.beats.simulator.utils.BeatsErrorLog;
import edu.berkeley.path.beats.simulator.utils.Table;

public class Controller_IRM_Traffic_Responsive extends Controller {
	
	private Link onramplink = null;
	private Link mainlinelink = null;
	private Sensor mainlinesensor = null;
	private Sensor queuesensor = null;
	
	boolean hasmainlinesensor; 		// true if config file contains entry for mainlinesensor
	boolean hasqueuesensor; 		// true if config file contains entry for queuesensor

	private boolean istablevalid;   // true if a valid table is given

	boolean hasoccthres;
	boolean hasflowthres;
	boolean hasspeedthres;
	
	private double[] trFlowThresh;  // stores flow thresholds corresponding to the traffic responsive controllers.
	private double[] trOccThresh;  // stores occupancy thresholds corresponding to the traffic responsive controllers.
	private double[] trSpeedThresh;  // stores speed thresholds corresponding to the traffic responsive controllers.
	private double[] trMeteringRates_normalized; // normalized metering rates corresponding to the different levels of the traffic responsive controller.
	
	private int trlevelindex; // denotes the current level that is requested by the traffic responsive logic.

	private Table table;
	
	/////////////////////////////////////////////////////////////////////
	// Construction
	/////////////////////////////////////////////////////////////////////

	public Controller_IRM_Traffic_Responsive(Scenario myScenario,edu.berkeley.path.beats.jaxb.Controller c) {
		super(myScenario,c,Algorithm.IRM_TOS);
	}

//	public Controller_IRM_Traffic_Responsive(Scenario myScenario,Link onramplink,Link mainlinelink,Sensor mainlinesensor,Sensor queuesensor,Table trtable){
//
//		this.myScenario = myScenario;
//		this.onramplink 	= onramplink;
//		this.mainlinelink 	= mainlinelink;
//		this.mainlinesensor = mainlinesensor;
//		this.queuesensor 	= queuesensor;
//		
//		hasmainlinelink   = mainlinelink!=null;
//		hasmainlinesensor = mainlinesensor!=null;
//		hasqueuesensor    = queuesensor!=null;
//		
//		// abort unless there is either one mainline link or one mainline sensor
//		if(mainlinelink==null && mainlinesensor==null)
//			return;
//		if(mainlinelink!=null  && mainlinesensor!=null)
//			return;
//		
//		usesensor = mainlinesensor!=null;
//		
//		// need the sensor's link for target density
//		if(usesensor)
//			mainlinelink = mainlinesensor.getMyLink();
//		
//		// Traffic responsive table.
//		this.table = trtable;
//		
//		this.extractTable();
//		
//	}
	
	/////////////////////////////////////////////////////////////////////
	// populate / validate / reset  / update
	/////////////////////////////////////////////////////////////////////

	@Override
	protected void populate(Object jaxbobject) {

		edu.berkeley.path.beats.jaxb.Controller jaxbc = (edu.berkeley.path.beats.jaxb.Controller) jaxbobject;
		
		if(jaxbc.getTargetActuators()==null || 
				   jaxbc.getTargetActuators().getTargetActuator()==null ||
				   jaxbc.getFeedbackSensors()==null ||
				   jaxbc.getFeedbackSensors().getFeedbackSensor()==null )
					return;			
		
//		hasmainlinelink = false;
		hasmainlinesensor = false;
		hasqueuesensor = false;
		
		// There should be only one target element, and it is the onramp
		if(jaxbc.getTargetActuators().getTargetActuator().size()==1){
			edu.berkeley.path.beats.jaxb.TargetActuator s = jaxbc.getTargetActuators().getTargetActuator().get(0);
			onramplink = getMyScenario().get.linkWithId(s.getId());
		}
		
		// Feedback elements can be "mainlinesensor","mainlinelink", and "queuesensor"
		if(!jaxbc.getFeedbackSensors().getFeedbackSensor().isEmpty()){
			
			for(edu.berkeley.path.beats.jaxb.FeedbackSensor s:jaxbc.getFeedbackSensors().getFeedbackSensor()){
				
				if(s.getUsage()==null)
					return;
				
				if( s.getUsage().equalsIgnoreCase("mainlinesensor") && mainlinesensor==null){
					mainlinesensor=getMyScenario().get.sensorWithId(s.getId());
					hasmainlinesensor = true;
				}

//				if( s.getUsage().equalsIgnoreCase("mainlinelink") &&
//					s.getType().equalsIgnoreCase("link") && mainlinelink==null){
//					mainlinelink=getMyScenario().getLinkWithId(s.getId());
//					hasmainlinelink = true;
//				}

				if( s.getUsage().equalsIgnoreCase("queuesensor") && queuesensor==null){
					queuesensor=getMyScenario().get.sensorWithId(s.getId());
					hasqueuesensor = true;
				}				
			}
		}
		
		// abort unless there is either one mainline link or one mainline sensor
		if(mainlinesensor==null)
			return;
//		if(mainlinelink==null && mainlinesensor==null)
//			return;
//		if(mainlinelink!=null  && mainlinesensor!=null)
//			return;
		
//		usesensor = mainlinesensor!=null;
		
		// need the sensor's link for target density
//		if(usesensor)
		mainlinelink = mainlinesensor.getMyLink();
		
		if(mainlinelink==null)
			return;	

		table = findTable(jaxbc, "tod");
		this.extractTable();
	}
	
	@Override
	protected void validate() {
		
		super.validate();
		
		// must have exactly one actuator
		if(getNumActuators()!=1)
			BeatsErrorLog.addError("Numnber of targets for traffic responsive controller ID=" + getId()+ " does not equal one.");

		// bad mainline sensor ID
		if(hasmainlinesensor && mainlinesensor==null)
			BeatsErrorLog.addError("Bad mainline sensor ID in traffic responsive controller ID=" + getId()+".");
		
		// bad queue sensor ID
		if(hasqueuesensor && queuesensor==null)
			BeatsErrorLog.addError("Bad queue sensor ID in traffic responsive controller ID=" + getId()+".");
		
		// Target link ID not found, or number of targets not 1.
		if(onramplink==null)
			BeatsErrorLog.addError("Invalid onramp link for traffic responsive controller ID=" + getId()+ ".");

		// both link and sensor feedback
//		if(hasmainlinelink && hasmainlinesensor)
//			BeatsErrorLog.addError("Both mainline link and mainline sensor are not allowed in traffic responsive controller ID=" + getId()+".");

		// sensor is disconnected
		if(mainlinesensor.getMyLink()==null)
			BeatsErrorLog.addError("Mainline sensor is not connected to a link in traffic responsive controller ID=" + getId()+ " ");

		// no feedback
		if(mainlinelink==null)
			BeatsErrorLog.addError("Invalid mainline link for traffic responsive controller ID=" + getId()+ ".");

		// Target link ID not found, or number of targets not 1.
		if(onramplink==null)
			BeatsErrorLog.addError("Invalid onramp link for traffic responsive controller ID=" + getId()+ ".");
			
		// invalid table
		if(!istablevalid)
			BeatsErrorLog.addError("Controller has an invalid table.");			
	}
	
	@Override
	protected void update() {
		
//		double mainlineocc=Double.POSITIVE_INFINITY;
//		double mainlinespeed=Double.POSITIVE_INFINITY;
//		double mainlineflow=Double.POSITIVE_INFINITY;
//
//		if (hasoccthres)
//			mainlineocc = mainlinesensor.getOccupancy(0);			
//		
//		if (hasspeedthres)
//			mainlinespeed = mainlinesensor.getSpeedInMPS(0);
//		
//		if (hasflowthres)
//			mainlineflow = mainlinesensor.getTotalFlowInVPS(0);		
//		
//		// metering rate adjustments
//		while (trlevelindex >0 && (hasoccthres && mainlineocc<=trOccThresh[trlevelindex]) 
//				&& (hasspeedthres && mainlinespeed<=trSpeedThresh[trlevelindex])
//				&& (hasflowthres && mainlineflow<=trFlowThresh[trlevelindex]))
//			trlevelindex--;
//		
//		while (trlevelindex <trMeteringRates_normalized.length-1 &&
//				((hasoccthres && mainlineocc>trOccThresh[trlevelindex+1]) || 
//				(hasspeedthres && mainlinespeed>trSpeedThresh[trlevelindex])
//				|| (hasflowthres && mainlineflow>trFlowThresh[trlevelindex])))
//			trlevelindex++;
//		
//		setControl_maxflow(0, trMeteringRates_normalized[trlevelindex]);

	}

	/////////////////////////////////////////////////////////////////////
	// private methods
	/////////////////////////////////////////////////////////////////////

	private void extractTable(){
		if (null == table) {
			istablevalid = false;
			return;
		}

		// read parameters from table, and also validate
		
		
		int rateIndx = table.getColumnNo("MeteringRates");
		int occIndx = table.getColumnNo("OccupancyThresholds");
		int spdIndx = table.getColumnNo("SpeedThresholds");
		int flwIndx = table.getColumnNo("FlowThresholds");
		
		hasflowthres=(flwIndx!=-1);
		hasspeedthres=(spdIndx!=-1);
		hasoccthres=(occIndx!=-1);		
		
//		istablevalid=table.checkTable() && (rateIndx!=-1) && (hasflowthres || hasoccthres || hasspeedthres);
		
		// need a valid table to parse
		if (!istablevalid) 
			return;
		
		
//		// read table, initialize values.
//		if (hasflowthres)
//			trFlowThresh=new double[table.getNoRows()];
//
//		if (hasoccthres)
//			trOccThresh=new double[table.getNoRows()];
//
//		if (hasspeedthres)
//			trSpeedThresh=new double[table.getNoRows()];
//
//
//		trMeteringRates_normalized=new double[table.getNoRows()];
//		trlevelindex = 0;
		// extract data from the table and populate
//		for (int i=0;i<table.getNoRows();i++){
//			trMeteringRates_normalized[i] = Double.parseDouble(table.getTableElement(i,rateIndx)) * getMyScenario().getSimdtinseconds(); // in veh per sim step
//			if (hasflowthres){
//				trFlowThresh[i]=Double.parseDouble(table.getTableElement(i,flwIndx));			// flow in veh/sec
//			}
//			if (hasoccthres){
//				trOccThresh[i]=Double.parseDouble(table.getTableElement(i,occIndx));  			// occupancy in %
//			}
//			if (hasspeedthres){
//				trSpeedThresh[i]=Double.parseDouble(table.getTableElement(i,spdIndx)); 			// speed in m/s
//			}
//
//			if (i==0 && ((hasflowthres && trFlowThresh[i]<0) || (hasoccthres && trOccThresh[i]<0) ||
//					(hasspeedthres && trSpeedThresh[i]<0)))
//					istablevalid=false;
//			// decreasing metering rates, and increasing thresholds, where applicable.		
//			if ((trMeteringRates_normalized[i]<0) || (i>0 && (trMeteringRates_normalized[i]>trMeteringRates_normalized[i-1])) || 
//			(i>0 && !((hasflowthres && trFlowThresh[i]>trFlowThresh[i-1]) || (hasoccthres && trOccThresh[i]>trOccThresh[i-1]) ||
//					(hasspeedthres && trSpeedThresh[i]>trSpeedThresh[i-1]))))				
//				istablevalid=false;					
//		}
		
		// occupancy thresholds should be between 0 and 100.
		if (hasoccthres && trOccThresh[0]<=0 && trOccThresh[trOccThresh.length-1]>100)
			istablevalid=false;
	}
	
}
