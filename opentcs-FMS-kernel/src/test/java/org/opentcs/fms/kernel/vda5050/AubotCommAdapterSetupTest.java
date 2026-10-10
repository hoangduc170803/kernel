package org.opentcs.fms.kernel.vda5050;

import static org.assertj.core.api.Assertions.assertThat;

import org.junit.jupiter.api.Test;
import org.opentcs.commadapter.vehicle.vda5050.v2_0.MqttSetting;
import org.opentcs.data.model.Vehicle;

class AubotCommAdapterSetupTest {

  private static Vehicle vda5050Vehicle(String serialNumber) {
    return new Vehicle("Vehicle-0001")
        .withProperty("vda5050:interfaceName", "aubotagv")
        .withProperty("vda5050:manufacturer", "AUBOT")
        .withProperty("vda5050:serialNumber", serialNumber)
        .withProperty("vda5050:version", "2.0");
  }

  @Test
  void buildsTheTopicPrefixTheVehiclesListenOn() {
    assertThat(AubotCommAdapter.agvTopicPrefix(vda5050Vehicle("B300_3_8")))
        .isEqualTo("aubotagv/2.0.0/AUBOT/B300_3_8");
  }

  @Test
  void doesNotUseTheStockVersionSegment() {
    assertThat(AubotCommAdapter.agvTopicPrefix(vda5050Vehicle("V01")))
        .doesNotContain("/v2/");
  }

  @Test
  void handsTheAubotTopicPrefixToTheStockAdapter() {
    assertThat(AubotCommAdapter.aubotMqttSetting(vda5050Vehicle("B300_3_8")).topicNamePrefix())
        .isEqualTo("aubotagv/2.0.0/AUBOT/B300_3_8");
  }

  @Test
  void subscribesAndPublishesOnTheTopicsTheVehiclesUse() {
    MqttSetting setting = AubotCommAdapter.aubotMqttSetting(vda5050Vehicle("B300_3_8"));

    assertThat(setting.stateTopicName()).isEqualTo("aubotagv/2.0.0/AUBOT/B300_3_8/state");
    assertThat(setting.connectionTopicName()).isEqualTo("aubotagv/2.0.0/AUBOT/B300_3_8/connection");
    assertThat(setting.orderTopicName()).isEqualTo("aubotagv/2.0.0/AUBOT/B300_3_8/order");
    assertThat(setting.instantActionsTopicName())
        .isEqualTo("aubotagv/2.0.0/AUBOT/B300_3_8/instantActions");
  }

  @Test
  void keepsTheVehicleTopicPrefixWhenTheModelDeclaresOne() {
    Vehicle vehicle = vda5050Vehicle("B300_3_8")
        .withProperty("vda5050:topicPrefix", "somewhere/else");

    assertThat(AubotCommAdapter.aubotMqttSetting(vehicle).topicNamePrefix())
        .isEqualTo("aubotagv/2.0.0/AUBOT/B300_3_8");
  }

  @Test
  void fallsBackToDefaultSpeedsWhenTheVehicleDeclaresNone() {
    Vehicle vehicle = vda5050Vehicle("V01");

    assertThat(AubotCommAdapter.speedProperty(
        vehicle, AubotCommAdapter.PROPKEY_SPEED_LOADED, AubotCommAdapter.DEFAULT_SPEED_LOADED))
        .isEqualTo(40.0);
    assertThat(AubotCommAdapter.speedProperty(
        vehicle, AubotCommAdapter.PROPKEY_SPEED_UNLOADED, AubotCommAdapter.DEFAULT_SPEED_UNLOADED))
        .isEqualTo(50.0);
  }

  @Test
  void readsSpeedsDeclaredOnTheVehicle() {
    Vehicle vehicle = vda5050Vehicle("V01")
        .withProperty(AubotCommAdapter.PROPKEY_SPEED_LOADED, " 35 ");

    assertThat(AubotCommAdapter.speedProperty(
        vehicle, AubotCommAdapter.PROPKEY_SPEED_LOADED, AubotCommAdapter.DEFAULT_SPEED_LOADED))
        .isEqualTo(35.0);
  }

  @Test
  void keepsDefaultSpeedWhenTheVehicleDeclaresGarbage() {
    Vehicle vehicle = vda5050Vehicle("V01")
        .withProperty(AubotCommAdapter.PROPKEY_SPEED_LOADED, "fast");

    assertThat(AubotCommAdapter.speedProperty(
        vehicle, AubotCommAdapter.PROPKEY_SPEED_LOADED, AubotCommAdapter.DEFAULT_SPEED_LOADED))
        .isEqualTo(40.0);
  }
}
