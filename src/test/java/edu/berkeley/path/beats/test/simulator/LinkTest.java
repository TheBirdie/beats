package edu.berkeley.path.beats.test.simulator;

import static org.junit.Assert.*;

import edu.berkeley.path.beats.Jaxb;
import org.junit.BeforeClass;
import org.junit.Test;

import edu.berkeley.path.beats.simulator.Defaults;
import edu.berkeley.path.beats.simulator.Link;
import edu.berkeley.path.beats.simulator.Scenario;

public class LinkTest {

	private static Scenario scenario;
	private static Link link;
	private static String config_folder = "data/config/";
	private static String config_file = "_smalltest.xml";
	
	@BeforeClass
	public static void setUpBeforeClass() throws Exception {
		scenario = Jaxb.create_scenario_from_xml(config_folder + config_file);
		if(scenario==null)
			fail("scenario did not load");
		
		link = scenario.get.linkWithId(-4);

		// initialize
		double timestep = Defaults.getTimestepFor(config_file);
		double starttime = 300d;
		double endtime = Double.POSITIVE_INFINITY;
		int numEnsemble = 1;
		scenario.initialize(timestep,starttime,endtime,numEnsemble);
		scenario.reset();
		
		// run the scenario
		scenario.advanceNSeconds(300d);

	}

	@Test
	public void test_getMyType() {
		assertTrue(link.isFreeway());
	}

	@Test
	public void test_getMyNetwork() {
		assertEquals(link.getMyNetwork().getId(),-1);
	}

	@Test
	public void test_getBegin_node() {
		assertEquals(link.getBegin_node().getId(),-4);
	}

	@Test
	public void test_getEnd_node() {
		assertEquals(link.getEnd_node().getId(),-5);
	}

	@Test
	public void test_getLengthInMeters() {
		double length_in_miles = 0.527494326813265;
		double expected = 1609.34*length_in_miles;
		assertEquals(link.getLengthInMeters(),expected,1e-2);
	}

	@Test
	public void test_get_Lanes() {
		assertEquals(link.get_Lanes(),1,1e-4);
	}	
	
	@Test
	public void test_isSource() {
		Link linksource = scenario.get.linkWithId(-6);
		assertFalse(link.isSource());
		assertTrue(linksource.isSource());
	}

	@Test
	public void test_isSink() {
		Link linksink = scenario.get.linkWithId(-7);
		assertFalse(link.isSink());
		assertTrue(linksink.isSink());
	}

	@Test
	public void test_getDensityInVeh_a() {

		double x = link.getDensityInVeh(0)[0]; 
		double exp = 15.008704188738106;
		
		assertEquals(exp,x,1e-4);

		// edge cases
		assertNull(link.getDensityInVeh(-1));
		assertNull(link.getDensityInVeh(100));
	}

	@Test
	public void test_getDensityInVeh_b() {

		double x = link.getDensityInVeh(0,0); 
		double exp = 15.008704188738106;
		
		assertEquals(exp,x,1e-4);
		
		// edge cases
		assertTrue(Double.isNaN(link.getDensityInVeh(-1,0)));
        assertTrue(Double.isNaN(link.getDensityInVeh(100,0)));
        assertTrue(Double.isNaN(link.getDensityInVeh(0,-1)));
	}
	
	@Test
	public void test_getTotalDensityInVeh() {

		double x = link.getTotalDensityInVeh(0); 
		double exp = 15.008704188738106;
		
		assertEquals(exp,x,1e-4);

		// edge cases
		assertTrue(Double.isNaN(link.getTotalDensityInVeh(-1)));
        assertTrue(Double.isNaN(link.getTotalDensityInVeh(100)));
	}

	@Test
	public void test_getTotalDensityInVPM() {

		double x = link.getTotalDensityInVPMeter(0); 
		double exp = 0.017679766286842164;
		
		assertEquals(exp,x,1e-4);
		
		// edge cases
		assertTrue(Double.isNaN(link.getTotalDensityInVPMeter(-1)));
        assertTrue(Double.isNaN(link.getTotalDensityInVPMeter(100)));
	}
	
	@Test
	public void test_getOutflowInVeh() {

		double x = link.getOutflowInVeh(0)[0]; 
		double exp = 0.39459704442691024;
		
		assertEquals(exp,x,1e-4);

		// edge cases 
		assertNull(link.getOutflowInVeh(-1));
		assertNull(link.getOutflowInVeh(100));
	}

	@Test
	public void test_getTotalOutflowInVeh() {

		double x = link.getTotalOutflowInVeh(0); 
		double exp = 0.39459704442691024;
		
		assertEquals(exp,x,1e-4);

		// edge cases
		assertTrue(Double.isNaN(link.getTotalOutflowInVeh(-1)));
        assertTrue(Double.isNaN(link.getTotalOutflowInVeh(100)));
	}

	@Test
	public void test_getInflowInVeh() {

		double x = link.getInflowInVeh(0)[0]; 
		double exp = 0.41666666666666663;
		
		assertEquals(exp,x,1e-4);

		// edge cases
		assertNull(link.getInflowInVeh(-1));
		assertNull(link.getInflowInVeh(100));
	}

	@Test
	public void test_getTotalInlowInVeh() {
		double x = link.getTotalInflowInVeh(0); 
		double exp = 0.41666666666666663;
		
		assertEquals(exp,x,1e-4);

		// edge cases
		assertTrue(Double.isNaN(link.getTotalInflowInVeh(-1)));
        assertTrue(Double.isNaN(link.getTotalInflowInVeh(100)));
	}

	@Test
	public void test_computeSpeedInMPS() {
		double x = link.computeSpeedInMPS(0); 
		double exp = 4.463826478527397;
		
		assertEquals(exp,x,1e-4);

		// edge cases
		assertTrue(Double.isNaN(link.computeSpeedInMPS(-1)));
		assertTrue(Double.isNaN(link.computeSpeedInMPS(100)));
	}

	@Test
	public void test_getDensityJamInVeh() {
		double x = link.getDensityJamInVeh(0); 
		double exp = 79.12414902198975;
		
		assertEquals(exp,x,1e-4);

		// edge cases
		assertTrue(Double.isNaN(link.getDensityJamInVeh(-1)));
		assertTrue(Double.isNaN(link.getDensityJamInVeh(100)));
	}
	
	@Test
	public void test_getDensityCriticalInVeh() {
		double x = link.getDensityCriticalInVeh(0); 
		double exp = 15.824829804397952;

		assertEquals(exp,x,1e-4);

		// edge cases
		assertTrue(Double.isNaN(link.getDensityCriticalInVeh(-1)));
		assertTrue(Double.isNaN(link.getDensityCriticalInVeh(100)));
	}

	@Test
	public void test_getCapacityDropInVeh() {
		double x = link.getCapacityDropInVeh(0);
		double exp = 0.0;
		
		assertEquals(exp,x,1e-4);
		
		// edge cases
		assertTrue(Double.isNaN(link.getCapacityDropInVeh(-1)));
		assertTrue(Double.isNaN(link.getCapacityDropInVeh(100)));
	}

	@Test
	public void test_getCapacityInVeh() {
		double x = link.getCapacityInVeh(0);
		double exp = 0.41666666666666663;

		assertEquals(exp,x,1e-4);
		
		// edge cases
		assertTrue(Double.isNaN(link.getCapacityInVeh(-1)));
		assertTrue(Double.isNaN(link.getCapacityInVeh(100)));
	}
	
	@Test
	public void test_getDensityJamInVPMPL() {
		double x = link.getDensityJamInVPMPL(0);
		double exp = 0.09320567883560009;

		assertEquals(exp,x,1e-4);
		
		// edge cases
		assertTrue(Double.isNaN(link.getDensityJamInVPMPL(-1)));
		assertTrue(Double.isNaN(link.getDensityJamInVPMPL(100)));
	}

	@Test
	public void test_getDensityCriticalInVPMPL() {
		double x = link.getDensityCriticalInVPMPL(0);
		double exp = 0.01864113576712002;

		assertEquals(exp,x,1e-4);
		
		// edge cases
		assertTrue(Double.isNaN(link.getDensityCriticalInVPMPL(-1)));
		assertTrue(Double.isNaN(link.getDensityCriticalInVPMPL(100)));
	}

	@Test
	public void test_getCapacityDropInVPSPL() {
		double x = link.getCapacityDropInVPSPL(0);
		double exp = 0.0;

		assertEquals(exp,x,1e-4);
		
		// edge cases
		assertTrue(Double.isNaN(link.getCapacityDropInVPSPL(-1)));
		assertTrue(Double.isNaN(link.getCapacityDropInVPSPL(100)));
	}

	@Test
	public void test_getCapacityInVPS() {
		double x = link.getCapacityInVPS(0);
		double exp = 0.08333333333333333;

		assertEquals(exp,x,1e-4);
		
		// edge cases
		assertTrue(Double.isNaN(link.getCapacityInVPS(-1)));
		assertTrue(Double.isNaN(link.getCapacityInVPS(100)));
	}

	@Test
	public void test_getCapacityInVPSPL() {
		double x = link.getCapacityInVPSPL(0);
		double exp = 0.08333333333333333;

		assertEquals(exp,x,1e-4);
		
		// edge cases
		assertTrue(Double.isNaN(link.getCapacityInVPSPL(-1)));
		assertTrue(Double.isNaN(link.getCapacityInVPSPL(100)));
	}

	@Test
	public void test_getNormalizedVf() {
		double x = link.getNormalizedVf(0);
		double exp = 0.026329930357346962;

		assertEquals(exp,x,1e-4);
		
		// edge cases
		assertTrue(Double.isNaN(link.getNormalizedVf(-1)));
		assertTrue(Double.isNaN(link.getNormalizedVf(100)));
	}

	@Test
	public void test_getVfInMPS() {
		double x = link.getVfInMPS(0);
		double exp = 4.4704;

		assertEquals(exp,x,1e-4);
		
		// edge cases
		assertTrue(Double.isNaN(link.getVfInMPS(-1)));
		assertTrue(Double.isNaN(link.getVfInMPS(100)));
	}
	
	@Test
	public void test_getCriticalSpeedInMPS() {
		double x = link.getCriticalSpeedInMPS(0);
		double exp = 4.4704;

		assertEquals(exp,x,1e-4);
		
		// edge cases
		assertTrue(Double.isNaN(link.getCriticalSpeedInMPS(-1)));
		assertTrue(Double.isNaN(link.getCriticalSpeedInMPS(100)));
	}

	@Test
	public void test_getNormalizedW() {
		double x = link.getNormalizedW(0);
		double exp = 0.006582482589336741;
		
		assertEquals(exp,x,1e-4);
		
		// edge cases
		assertTrue(Double.isNaN(link.getNormalizedW(-1)));
		assertTrue(Double.isNaN(link.getNormalizedW(100)));
	}

	@Test
	public void test_getWInMPS() {
		double x = link.getWInMPS(0);
		double exp = 1.1176;

		assertEquals(exp,x,1e-4);
		
		// edge cases
		assertTrue(Double.isNaN(link.getWInMPS(-1)));
		assertTrue(Double.isNaN(link.getWInMPS(100)));
	}

	@Test
	public void test_overrideDensityWithVeh() {
		double [] olddensity = link.getDensityInVeh(0);
		double [] newdensity = link.getDensityInVeh(0);
		newdensity[0] *= 2d;
		link.overrideDensityWithVeh(newdensity,0);

		assertEquals(link.getDensityInVeh(0,0) , olddensity[0]*2d , 1e-4 );

		link.overrideDensityWithVeh(olddensity,0);
	}
	
	@Test
	public void test_getDensity() {
		double x = link.getDensityInVeh(0)[0];
		double exp = 15.008704188738106;
		
		assertEquals(exp,x,1e-4);

		// edge case
		assertNull(link.getDensityInVeh(-1));
		assertNull(link.getDensityInVeh(100));
	}

	@Test
	public void test_getInputFlow() {
		double x = link.getInflowInVeh(0,0);
		double exp = 0.41666666666666663;
		assertEquals(exp,x,1e-4);
		
		// edge cases
		assertTrue(Double.isNaN(link.getInflowInVeh(-1,0)));
		assertTrue(Double.isNaN(link.getInflowInVeh(100,0)));
		assertTrue(Double.isNaN(link.getInflowInVeh(0,-1)));
		assertTrue(Double.isNaN(link.getInflowInVeh(0,100)));
	}

	@Test
	public void test_getOutputFlow() {
		double x = link.getOutflowInVeh(0,0);
		double exp = 0.39459704442691024;
		assertEquals(exp,x,1e-4);

		// edge cases
		assertTrue(Double.isNaN(link.getOutflowInVeh(-1,0)));
		assertTrue(Double.isNaN(link.getOutflowInVeh(100,0)));
		assertTrue(Double.isNaN(link.getOutflowInVeh(0,-1)));
		assertTrue(Double.isNaN(link.getOutflowInVeh(0,100)));
	}
}
