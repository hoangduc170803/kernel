package org.opentcs.fms.routing;

/** Shared switches for selecting a local-guidance implementation. */
final class GuideSettings {

  static final String FMS_PROFILE = "fms";
  static final String LLLG_PROFILE = "lllg";

  private GuideSettings() {
  }

  static boolean enabled() {
    return Boolean.parseBoolean(System.getProperty("fms.guide.on", "true"));
  }

  static String profile() {
    return System.getProperty("fms.guide.profile", FMS_PROFILE);
  }
}
