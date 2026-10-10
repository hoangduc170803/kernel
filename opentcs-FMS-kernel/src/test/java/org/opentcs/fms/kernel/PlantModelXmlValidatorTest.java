package org.opentcs.fms.kernel;

import static org.assertj.core.api.Assertions.assertThat;

import org.junit.jupiter.api.Test;

class PlantModelXmlValidatorTest {

  private static final String BOX
      = "<maxVehicleBoundingBox length=\"1000\" width=\"1000\" height=\"1000\""
      + " referenceOffsetX=\"0\" referenceOffsetY=\"0\"/>";

  private final PlantModelXmlValidator validator = new PlantModelXmlValidator();

  private static String model(String version, String body) {
    return "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
        + "<model version=\"" + version + "\" name=\"factory\">\n"
        + body
        + "  <visualLayout name=\"layout\" scaleX=\"50.0\" scaleY=\"50.0\">\n"
        + "    <layer id=\"0\" ordinal=\"0\" visible=\"true\" name=\"L\" groupId=\"0\"/>\n"
        + "    <layerGroup id=\"0\" name=\"G\" visible=\"true\"/>\n"
        + "  </visualLayout>\n"
        + "</model>\n";
  }

  private static String point(String name) {
    return "  <point name=\"" + name + "\" positionX=\"0\" positionY=\"0\" positionZ=\"0\""
        + " type=\"HALT_POSITION\">" + BOX + "</point>\n";
  }

  private static String location(String lockedAttribute) {
    return "  <locationType name=\"Charge\"><allowedOperation name=\"startCharging\"/>"
        + "</locationType>\n"
        + "  <location name=\"L1\" type=\"Charge\" positionX=\"0\" positionY=\"500\""
        + " positionZ=\"0\"" + lockedAttribute + "><link point=\"P1\"/></location>\n";
  }

  @Test
  void acceptsAModelTheModelEditorWrites() {
    PlantModelXmlValidator.Result result
        = validator.validate(model("7.0.0", point("P1") + location(" locked=\"false\"")));

    assertThat(result.valid()).isTrue();
    assertThat(result.version()).isEqualTo("7.0.0");
    assertThat(result.message()).isNull();
  }

  @Test
  void reportsTheSchemaViolationWithItsPosition() {
    PlantModelXmlValidator.Result result
        = validator.validate(model("7.0.0", point("P1") + location("")));

    assertThat(result.valid()).isFalse();
    assertThat(result.message()).contains("locked").contains("location");
    assertThat(result.line()).isEqualTo(5);
    assertThat(result.column()).isPositive();
  }

  @Test
  void rejectsElementsOutOfSchemaOrder() {
    PlantModelXmlValidator.Result result = validator.validate(
        model("7.0.0", location(" locked=\"false\"") + point("P1"))
    );

    assertThat(result.valid()).isFalse();
    assertThat(result.line()).isNotNull();
  }

  @Test
  void rejectsAModelWithoutVersion() {
    PlantModelXmlValidator.Result result
        = validator.validate("<model name=\"factory\"></model>");

    assertThat(result.valid()).isFalse();
    assertThat(result.message()).contains("version");
  }

  @Test
  void rejectsMalformedXml() {
    PlantModelXmlValidator.Result result
        = validator.validate("<model version=\"7.0.0\" name=\"x\">\n<point>");

    assertThat(result.valid()).isFalse();
    assertThat(result.message()).isNotBlank();
  }
}
