package edu.berkeley.path.beats.control;

import edu.berkeley.path.beats.actuator.ActuatorVehType;
import edu.berkeley.path.beats.simulator.*;
import edu.berkeley.path.beats.simulator.utils.BeatsException;
import edu.berkeley.path.beats.simulator.utils.Table;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;

/**
 * Created by matt on 4/20/16.
 */
public class Controller_HOT_Lookup extends Controller {

	private HashMap<Long, LinkData> linkData;

	public Controller_HOT_Lookup(Scenario scenario, edu.berkeley.path.beats.jaxb.Controller c) {
		super(scenario, c, Algorithm.HOT_LOOKUP);
		// the superclass constructor prepares the tables
		// column names for the tables: HOT lane flow, HOT lane speed, ML speed
		// properties for the tables: 	GP Link, HOT Link, FF Price Coefficient, FF Intercept, VehTypeIn, VehTypeOut,
		// 								Congested Price Coefficient, Congested GP Density Coefficient, Congested Intercept,
		//								Start time, Stop time
	}

	@Override
	public void populate(Object jaxbO) {
		super.populate(jaxbO);

		// create the LinkData objects
		for (Table table : tables.values()) {
			Long linkid = Long.valueOf(table.getParameters().get("GP Link"));
			if (linkData.containsKey(linkid))
				linkData.get(linkid).addTable(table);
			else {
				Link link = myScenario.get.linkWithId(linkid);
				linkData.put(linkid, new LinkData(link, table, this));
			}
		}
	}

	@Override
	public boolean register() {
		for(LinkData ld : linkData.values())
			if(!ld.myActuator.register())
				return false;
		return true;
	}

	@Override
	protected void validate() {
		super.validate();

		for (LinkData ld : linkData.values())
			ld.validate();
	}

	@Override
	protected void reset() {

		for (LinkData ld : linkData.values())
			ld.reset();
	}

	protected void update(Clock clock) throws BeatsException {

		for (LinkData ld : linkData.values())
			ld.update(clock);

	}

	class LinkData {

		protected Link myGPLink;
		protected ActuatorVehType myActuator;
		protected Link myHOTLink;

		protected List<Table> tables;
		protected List<Double> startTimes;
		protected List<Double> stopTimes;
		protected List<Integer> vehTypeIn;
		protected List<Integer> vehTypeOut;
		protected List<Double> FF_intercept;
		protected List<Double> FF_price_coeff;
		protected List<Double> Cong_price_coeff;
		protected List<Double> Cong_GP_density_coeff;
		protected List<Double> Cong_intercept;

		private List<Boolean> isActive;
		private List<Table> currentTables;
		private List<List<Double>> currentPrices; // vehtype x ensemble index

		public LinkData(Link link, Table T, Controller parent) {

			this.myGPLink = link;

			Long HOTLinkId = Long.valueOf(T.getParameters().get("HOT Link"));
			this.myHOTLink = myScenario.get.linkWithId(HOTLinkId);

			startTimes = new ArrayList<Double>();
			stopTimes = new ArrayList<Double>();
			vehTypeIn = new ArrayList<Integer>();
			vehTypeOut = new ArrayList<Integer>();
			FF_intercept = new ArrayList<Double>();
			FF_price_coeff = new ArrayList<Double>();
			Cong_price_coeff = new ArrayList<Double>();
			Cong_GP_density_coeff = new ArrayList<Double>();
			Cong_intercept = new ArrayList<Double>();
			isActive = new ArrayList<Boolean>();

			currentTables = new ArrayList<Table>();


			for(int e=0;e<myScenario.get.numEnsemble(); e++)
				currentPrices.add(new ArrayList<Double>());

			addTable(T);

			// make actuator
			edu.berkeley.path.beats.jaxb.Actuator jaxbA = new edu.berkeley.path.beats.jaxb.Actuator();
			edu.berkeley.path.beats.jaxb.ScenarioElement se = new edu.berkeley.path.beats.jaxb.ScenarioElement();
			edu.berkeley.path.beats.jaxb.ActuatorType at = new edu.berkeley.path.beats.jaxb.ActuatorType();
			se.setId(myGPLink.getId());
			se.setType("link");
			at.setId(-1);
			at.setName("vehtype_changer");
			jaxbA.setId(-1);
			jaxbA.setScenarioElement(se);
			jaxbA.setActuatorType(at);
			myActuator = new ActuatorVehType(myScenario,jaxbA,new BeatsActuatorImplementation(jaxbA,myScenario));
			myActuator.populate(null,null);
			myActuator.setMyController(parent);
		}

		private void addTable(Table T) {
			if(!tables.contains(T)) {
				tables.add(T);

				startTimes.add( Double.valueOf(T.getParameters().get("Start Time")));
				stopTimes.add( Double.valueOf(T.getParameters().get("Stop Time")));
				vehTypeIn.add( Integer.valueOf(T.getParameters().get("VehTypeIn")));
				vehTypeOut.add( Integer.valueOf(T.getParameters().get("VehTypeOut")));
				FF_intercept.add( Double.valueOf(T.getParameters().get("FF Intercept")));
				FF_price_coeff.add( Double.valueOf(T.getParameters().get("FF Price Coefficient")));
				Cong_price_coeff.add(Double.valueOf(T.getParameters().get("Congested Price Coefficient")));
				Cong_GP_density_coeff.add(Double.valueOf(T.getParameters().get("Congested GP Density Coefficient")));
				Cong_intercept.add(Double.valueOf(T.getParameters().get("Congested Intercept")));
				isActive.add(false);

				double t = myScenario.get.currentTimeInSeconds();
				if ((startTimes.get(startTimes.size()-1) >= t)	&& (stopTimes.get(stopTimes.size()-1) <= t) ) {
					if (currentTables.isEmpty() || isTableActivatable(T))
						activateTable(T);
				}
			}
		}

		private boolean isTableActivatable(Table table) {
			for (int i=0; i<isActive.size(); i++) {
				if(isActive.get(i) && (vehTypeIn.get(i).equals( vehTypeIn.get( tables.indexOf(table) ) )))
					return false;
			}
			return true;
		}

		private void activateTable(Table table) {
			currentTables.add(table);
			isActive.set( tables.indexOf(table), true );
		}


		private void update(Clock clock) {
			updateTables(clock);
			updatePrices(clock);
		}

		private void updateTables(Clock clock) {
			Table table;
			Iterator<Table> i = currentTables.iterator();
			while (i.hasNext()) {
				table = i.next();
				if( clock.getT() > stopTimes.get( tables.indexOf(table) ) ) {
					currentVehTypesIn.remove( tables.indexOf(table) );
					for (int e=0; e<currentPrices.size(); e++ ) {
						currentPrices.get(e)
					}
					i.remove();
				}
			}
		}

		private void updatePrices(Clock clock) {

		}
		private double computeCurrentPrice(Table table, Clock clock, int ensembleIndex) {
			if(myGPLink.getDensityCriticalInVeh())
		}

	}

}
