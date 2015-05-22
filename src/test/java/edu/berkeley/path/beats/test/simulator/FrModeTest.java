package edu.berkeley.path.beats.test.simulator;

import edu.berkeley.path.beats.Jaxb;
import edu.berkeley.path.beats.Runner;
import edu.berkeley.path.beats.simulator.Defaults;
import edu.berkeley.path.beats.simulator.DemandProfile;
import edu.berkeley.path.beats.simulator.DemandSet;
import edu.berkeley.path.beats.simulator.Scenario;
import edu.berkeley.path.beats.simulator.utils.BeatsException;
import org.junit.BeforeClass;
import org.junit.Ignore;
import org.junit.Test;

import static org.junit.Assert.fail;

/**
 * Created by gomes on 5/11/2015.
 */
public class FrModeTest {




    @Ignore
    @Test
    public void testWithProperties() throws Exception {
        try {
            //String config_file = "C:\\Users\\gomes\\Dropbox\\_work_other\\polytechnique\\calibration\\config\\210E_joined_frmode.xml";

            String propsfile = "C:\\Users\\Felix\\code\\autoCalibrationProject\\config\\frmode.properties";
            int link_id = -24026218;
            Scenario scenario = Runner.load_scenario_from_properties(propsfile);
//            DemandSet demandSet = (DemandSet) scenario.getDemandSet();
//            DemandProfile dp = demandSet.get_demand_profile_for_link_id((long) link_id);

            scenario.reset();

//            System.out.println(dp._knob + "\t" + dp.getCurrentValue(0)[0]);

            scenario.set.knob_for_offramp_link_id(link_id,2);

//            System.out.println(dp._knob + "\t" + dp.getCurrentValue(0)[0]);


            // run the scenario
            scenario.run();

        } catch (BeatsException e) {
            fail("initialization failure.");
        }
    }
}
