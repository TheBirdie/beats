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

package edu.berkeley.path.beats.sensor;

import java.util.ArrayList;

import org.apache.log4j.Logger;

import edu.berkeley.path.beats.data.FiveMinuteData;
import edu.berkeley.path.beats.simulator.utils.BeatsErrorLog;
import edu.berkeley.path.beats.simulator.utils.BeatsMath;
import edu.berkeley.path.beats.simulator.Scenario;
import edu.berkeley.path.beats.simulator.Sensor;

public class SensorLoopStation extends edu.berkeley.path.beats.simulator.Sensor {
	
	private int VDS;								// PeMS vehicle detector station number
	private ArrayList<edu.berkeley.path.beats.sensor.DataSource> _datasources = new ArrayList<edu.berkeley.path.beats.sensor.DataSource>();
	private FiveMinuteData data;
	
	private Double [] cumulative_inflow;	// [veh] 	numEnsemble
	private Double [] cumulative_outflow;	// [veh] 	numEnsemble
	       
	private static Logger logger = Logger.getLogger(SensorLoopStation.class);

	/////////////////////////////////////////////////////////////////////
	// Construction
	/////////////////////////////////////////////////////////////////////

	public  SensorLoopStation(){
	}

	public  SensorLoopStation(Scenario myScenario,edu.berkeley.path.beats.jaxb.Sensor s,Sensor.Type myType){
		super(myScenario,s,myType);
	}
	
//	public SensorLoopStation(Scenario myScenario,String linkId){
//		if(myScenario==null)
//			return;
//		this.myScenario  = myScenario;
//	    this.myType = Sensor.Type.loop;
//	    this.myLink = myScenario.getLinkWithId(linkId);
//	}

	/////////////////////////////////////////////////////////////////////
	// populate / validate / reset / update
	/////////////////////////////////////////////////////////////////////	

	@Override
	protected void populate(Object jaxbobject) {
		
		edu.berkeley.path.beats.jaxb.Sensor jaxbs = (edu.berkeley.path.beats.jaxb.Sensor) jaxbobject;

		if(jaxbs.getParameters()!=null)
			for(edu.berkeley.path.beats.jaxb.Parameter param : jaxbs.getParameters().getParameter()){
				if(param.getName().compareToIgnoreCase("vds")==0)
					this.VDS = Integer.parseInt(param.getValue());
			}
		
//		if (null != jaxbs.getTable()) {
//			if ("data_sources".equals(jaxbs.getTable().getName())) {
//				edu.berkeley.path.beats.simulator.utils.Table table = new edu.berkeley.path.beats.simulator.utils.Table(jaxbs.getTable());
//				final String[] colname = {"url", "format"};
//				int[] colnum = new int[2];
//				boolean colnotfound = false;
//				for (int i = 0; i < colname.length; ++i)
//					if (-1 == (colnum[i] = table.getColumnNo(colname[i]))) {
//						logger.warn("Column '" + colname[i] + "' not found");
//						colnotfound = true;
//					}
//				if (!colnotfound)
//					for (int i = 0; i < table.getNoRows(); ++i)
//						this._datasources.add(new DataSource(table.getTableElement(i, colnum[0]), table.getTableElement(i, colnum[1])));
//			} else
//				logger.warn("sensor " + jaxbs.getId() + ": table name: " + jaxbs.getTable().getName());
//		}
	}

	@Override
	protected void validate() {
		if(getMyLink()==null)
			BeatsErrorLog.addWarning("Loop sensor with ID=" + getId() +" is not attached.");
	}

	@Override
	protected void reset() {
		int numEnsemble = getMyScenario().get.numEnsemble();
		cumulative_inflow = BeatsMath.zeros(numEnsemble);
		cumulative_outflow = BeatsMath.zeros(numEnsemble);
	}

	@Override
	protected void update() {		
		if(getMyLink()==null)
			return;
		for(int i=0;i<this.getMyScenario().get.numEnsemble();i++){
			cumulative_inflow[i] += getMyLink().getTotalInflowInVeh(i);
			cumulative_outflow[i] += getMyLink().getTotalOutflowInVeh(i);
		}
		return;
	}

	/////////////////////////////////////////////////////////////////////
	// InterfaceSensor
	/////////////////////////////////////////////////////////////////////
	
	@Override
	public double [] getDensityInVPM(int ensemble) {
		return BeatsMath.times(getMyLink().getDensityInVeh(ensemble), 1d / getMyLink().getLengthInMeters());
	}

	@Override
	public double getTotalDensityInVeh(int ensemble) {
		return getMyLink().getTotalDensityInVeh(ensemble);
	}
	
	@Override
	public double getTotalDensityInVPM(int ensemble) {
		return getMyLink().getTotalDensityInVeh(ensemble) / getMyLink().getLengthInMeters();
	}

//	@Override
//	public double getOccupancy(int ensemble) {
//		return myLink.getTotalDensityInVeh(ensemble)/myLink.getLengthInMiles()/this.getRho_jam()*100;
//	}
	
	@Override
	public double[] getFlowInVPS(int ensemble) {
		return BeatsMath.times(getMyLink().getOutflowInVeh(ensemble), 1 / getMyScenario().get.simdtinseconds());
	}

	@Override
	public double getTotalFlowInVPS(int ensemble) {
		return getMyLink().getTotalOutflowInVeh(ensemble) / getMyScenario().get.simdtinseconds();
	}

	@Override
	public double getSpeedInMPS(int ensemble) {
		return getMyLink().computeSpeedInMPS(ensemble);
	}

	/////////////////////////////////////////////////////////////////////
	// SensorLoopStation API
	/////////////////////////////////////////////////////////////////////

	public double getCumulativeInflowInVeh(int ensemble){
		return cumulative_inflow[ensemble];
	}

	public void resetCumulativeInflowInVeh(){
		for(int i=0;i<getMyScenario().get.numEnsemble();i++)
			cumulative_inflow[i] = 0d;
	}
	
	public double getCumulativeOutflowInVeh(int ensemble){
		return cumulative_outflow[ensemble];
	}

	public void resetCumulativeOutflowInVeh(){
		for(int i=0;i<getMyScenario().get.numEnsemble();i++)
			cumulative_outflow[i] = 0d;
	}
	
	public int getVDS() {
		return VDS;
	}

	public ArrayList<edu.berkeley.path.beats.sensor.DataSource> get_datasources() {
		return _datasources;
	}
	
	/////////////////////////////////////////////////////////////////////
	// data
	/////////////////////////////////////////////////////////////////////
	
	public void set5minData(FiveMinuteData indata){
		data = indata;
	}
	
	public int getNumDataPoints(){
		return data==null ? 0 : data.getNumDataPoints();
	}
	
	public ArrayList<Long> getDataTime(){
		return data==null ? null : data.getTime();
	}

	/** get aggregate flow value in [veh/sec]
	 * @param i time index
	 * @return a float, or <code>NaN</code> if something goes wrong.
	 * */
	public float getDataAggFlwInVPS(int i){
		return data==null ? Float.NaN : data.getAggFlwInVPS(i);
	}

	/** get aggregate flow vlaue in [veh/sec/lane]
	 * @param i time index
	 * @return a float, or <code>NaN</code> if something goes wrong.
	 * */
	public float getDataAggFlwInVPSPL(int i){
		return data==null ? Float.NaN : data.getAggFlwInVPSPL(i);
	}
	
	/** get aggregate speed value in [meters/sec]
	 * @param i time index
	 * @return a float, or <code>NaN</code> if something goes wrong.
	 * */
	public float getDataAggSpdInMPS(int i){
		return data==null ? Float.NaN : data.getAggSpd(i);

	}	

	/** get aggregate density value in [veh/meter]
	 * @param i time index
	 * @return a float, or <code>NaN</code> if something goes wrong.
	 * */
	public float getDataAggDtyInVPM(int i){
		return data==null ? Float.NaN : data.getAggDtyInVPM(i);
	}

	/** get aggregate density value in [veh/meter]
	 * @param i time index
	 * @return a float, or <code>NaN</code> if something goes wrong.
	 * */
	public float getDataAggDtyInVPMPL(int i){
		return data==null ? Float.NaN : data.getAggDtyInVPMPL(i);
	}

	//////////////////////////////
	
}
