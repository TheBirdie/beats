package edu.berkeley.path.beats.test.simulator;

import static org.junit.Assert.*;

import edu.berkeley.path.beats.Jaxb;
import org.junit.BeforeClass;
import org.junit.Test;

import edu.berkeley.path.beats.simulator.Defaults;
import edu.berkeley.path.beats.simulator.Scenario;
import edu.berkeley.path.beats.simulator.Sensor;

public class SensorTest {

	private static Scenario scenario;
	private static Sensor sensor;
	private static String config_folder = "data/config/";
	private static String config_file = "complete.xml";
	
	@BeforeClass
	public static void setUpBeforeClass() throws Exception {
		scenario = Jaxb.create_scenario_from_xml(config_folder + config_file);
		if(scenario==null)
			fail("scenario did not load");
		
		double timestep = Defaults.getTimestepFor(config_file);
		double starttime = 0;
		double endtime = 300;
		int numEnsemble = 1;
		scenario.initialize(timestep,starttime,endtime,numEnsemble);
		
		sensor = scenario.getSensorWithId(1);
	}

	@Test
	public void test_getMyType() {
		assertTrue(sensor.getMyType()==Sensor.Type.loop);
	}

	@Test
	public void test_getMyLink() {
		assertEquals(sensor.getMyLink().getId(),1);
	}

}
