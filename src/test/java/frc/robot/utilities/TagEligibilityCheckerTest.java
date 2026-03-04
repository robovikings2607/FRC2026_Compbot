package frc.robot.utilities;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.AfterEach;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.hal.HAL;

public class TagEligibilityCheckerTest {


    // This runs ONCE before any classes are fully instantiated
    @BeforeAll
    static void setupHAL() {
        assert HAL.initialize(500, 0); 
    }    

    private void setAllianceStation(AllianceStationID allianceStationID) {
        DriverStationSim.setAllianceStationId(allianceStationID);
        DriverStationSim.notifyNewData();        
    }


    @Test
    public void testRedAllianceIsEligibleTagIdFalse() {
        setAllianceStation(AllianceStationID.Red1);
        
        assertEquals(false, TagEligibilityChecker.INSTANCE.isEligibleTagId(99));
    }

    @Test
    public void testRedAllianceIsEligibleTagIdTrue() {
        setAllianceStation(AllianceStationID.Red1);
        
        assertEquals(true, TagEligibilityChecker.INSTANCE.isEligibleTagId(2));
    }

    @Test
    public void testBlueAllianceIsEligibleTagIdFalse() {
        setAllianceStation(AllianceStationID.Blue1);
        
        assertEquals(false, TagEligibilityChecker.INSTANCE.isEligibleTagId(99));
    }

    @Test
    public void testBlueAllianceIsEligibleTagIdTrue() {
        setAllianceStation(AllianceStationID.Blue1);
        
        assertEquals(true, TagEligibilityChecker.INSTANCE.isEligibleTagId(25));
    }

    @Test
    public void testRedAllianceWithBlueTagIdFalse() {
        setAllianceStation(AllianceStationID.Red1);
        
        assertEquals(false, TagEligibilityChecker.INSTANCE.isEligibleTagId(32));
    }

    @Test
    public void testBlueHubAllianceIsEligibleTagIdFalse() {
        setAllianceStation(AllianceStationID.Blue1);
        
        assertEquals(false, TagEligibilityChecker.INSTANCE.isEligibleHubTagId(17));
    }

    @Test
    public void testBlueHubAllianceIsEligibleTagIdTrue() {
        setAllianceStation(AllianceStationID.Blue1);
        
        assertEquals(true, TagEligibilityChecker.INSTANCE.isEligibleHubTagId(24));
    }

    @Test
    public void testRedHubAllianceIsEligibleTagIdFalse() {
        setAllianceStation(AllianceStationID.Red1);
        
        assertEquals(false, TagEligibilityChecker.INSTANCE.isEligibleHubTagId(17));
    }

    @Test
    public void testRedHubAllianceIsEligibleTagIdTrue() {
        setAllianceStation(AllianceStationID.Red1);
        
        assertEquals(true, TagEligibilityChecker.INSTANCE.isEligibleHubTagId(11));
    }

    @Test
    public void testRedHubAllianceIsEligibleShootingTagIdFalse() {
        setAllianceStation(AllianceStationID.Red1);
        
        assertEquals(false, TagEligibilityChecker.INSTANCE.isEligibleHubShootingTagId(25));
    }

    @Test
    public void testRedHubAllianceIsEligibleShootingTagIdTrue() {
        setAllianceStation(AllianceStationID.Red1);
        
        assertEquals(true, TagEligibilityChecker.INSTANCE.isEligibleHubShootingTagId(10));
    }
    @Test
    public void testBlueHubAllianceIsEligibleShootingTagIdFalse() {
        setAllianceStation(AllianceStationID.Blue1);
        
        assertEquals(false, TagEligibilityChecker.INSTANCE.isEligibleHubShootingTagId(16));
    }

    @Test
    public void testBlueHubAllianceIsEligibleShootingTagIdTrue() {
        setAllianceStation(AllianceStationID.Blue1);
        
        assertEquals(true, TagEligibilityChecker.INSTANCE.isEligibleHubShootingTagId(26));
    }

}
