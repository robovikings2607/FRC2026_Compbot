package frc.robot.utilities;

import java.util.Arrays;
import java.util.Set;
import java.util.stream.Collectors;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public enum TagEligibilityChecker {
  INSTANCE;

  //Tag Ids are for 2026 Rebuilt competition
  private int[] ALL_RED_TAG_IDS = new int[] {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
  private int[] ALL_BLUE_TAG_IDS = new int[] {17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32}; 
  private int[] RED_HUB_TAG_IDS = new int[] {11,2,3,4,5,8,9,10};     
  private int[] BLUE_HUB_TAG_IDS = new int[] {21,24,25,26,18,27,19,20};   
  private int[] RED_HUB_SHOOTING_TAG_IDS = new int[] {9,10};     
  private int[] BLUE_HUB_SHOOTIN_TAG_IDS = new int[] {25,26};   

  private final Set<Integer> allRedTagSet;  
  private final Set<Integer> allBlueTagSet;
  private final Set<Integer> redHubTagSet;      
  private final Set<Integer> blueHubTagSet;    
  private final Set<Integer> redHubShootingTagSet;      
  private final Set<Integer> blueHubShootingTagSet;    


  // The constructor is inherently private in an enum
  TagEligibilityChecker() {
    allBlueTagSet = loadTagSet(ALL_BLUE_TAG_IDS);
    allRedTagSet = loadTagSet(ALL_RED_TAG_IDS);
    blueHubTagSet = loadTagSet(BLUE_HUB_TAG_IDS);
    redHubTagSet = loadTagSet(RED_HUB_TAG_IDS);    
    redHubShootingTagSet = loadTagSet(RED_HUB_SHOOTING_TAG_IDS);
    blueHubShootingTagSet = loadTagSet(BLUE_HUB_SHOOTIN_TAG_IDS);            
  }

  private Set<Integer> loadTagSet(int[] tagIds) {
    return Arrays.stream(tagIds)
      .boxed()
      .collect(Collectors.toSet());
  }

  public boolean isEligibleTagId(int tagId) {
    boolean isEligibleTagId = false;

    if (DriverStation.getAlliance().isPresent()) {
      isEligibleTagId = DriverStation.getAlliance().get() == Alliance.Blue ? 
          allBlueTagSet.contains(tagId) : allRedTagSet.contains(tagId);
    }

    return isEligibleTagId;
  }

  public boolean isEligibleHubTagId(int tagId) {
    boolean isEligibleTagId = false;

    if (DriverStation.getAlliance().isPresent()) {
      isEligibleTagId = DriverStation.getAlliance().get() == Alliance.Blue ? 
         blueHubTagSet.contains(tagId) : redHubTagSet.contains(tagId);
    }

    return isEligibleTagId;
  }

  public boolean isEligibleHubShootingTagId(int tagId) {
    boolean isEligibleTagId = false;

    if (DriverStation.getAlliance().isPresent()) {
      isEligibleTagId = DriverStation.getAlliance().get() == Alliance.Blue ? 
         blueHubShootingTagSet.contains(tagId) : redHubShootingTagSet.contains(tagId);
    }

    return isEligibleTagId;
  }

}