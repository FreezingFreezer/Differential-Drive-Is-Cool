package robot;

import static lib.UnitTestingUtil.reset;

import org.junit.jupiter.api.Test;

import robot.drive.Robot;

public class RobotTest {
  @Test
  void initialize() throws Exception {
    new Robot().close();
    reset();
  }
}
