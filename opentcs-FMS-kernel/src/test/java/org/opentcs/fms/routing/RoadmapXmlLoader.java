package org.opentcs.fms.routing;

import java.io.File;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import org.opentcs.fms.kernel.VehicleOperations;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

public final class RoadmapXmlLoader {

  private RoadmapXmlLoader() {
  }

  public static Roadmap.Model load(File xml) throws Exception {
    DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
    factory.setNamespaceAware(false);
    factory.setValidating(false);
    try {
      factory.setFeature("http://apache.org/xml/features/nonvalidating/load-external-dtd", false);
    } catch (Exception ignored) {
    }
    DocumentBuilder builder = factory.newDocumentBuilder();
    var doc = builder.parse(xml);
    doc.getDocumentElement().normalize();

    List<Roadmap.Point> points = new ArrayList<>();
    NodeList pts = doc.getElementsByTagName("point");
    for (int i = 0; i < pts.getLength(); i++) {
      Element e = (Element) pts.item(i);
      if (!e.hasAttribute("positionX")) {
        continue; // not a real point definition
      }
      points.add(new Roadmap.Point(
          e.getAttribute("name"),
          Double.parseDouble(e.getAttribute("positionX")),
          Double.parseDouble(e.getAttribute("positionY")),
          e.hasAttribute("positionZ") ? Double.parseDouble(e.getAttribute("positionZ")) : 0.0));
    }

    List<Roadmap.Path> paths = new ArrayList<>();
    NodeList pas = doc.getElementsByTagName("path");
    for (int i = 0; i < pas.getLength(); i++) {
      Element e = (Element) pas.item(i);
      if (!e.hasAttribute("sourcePoint") || !e.hasAttribute("destinationPoint")) {
        continue;
      }
      double length = e.hasAttribute("length") ? Double.parseDouble(e.getAttribute("length")) : 1.0;
      boolean bidir = !e.hasAttribute("maxReverseVelocity")
          || Double.parseDouble(e.getAttribute("maxReverseVelocity")) > 0.0;
      String src = e.getAttribute("sourcePoint");
      String dst = e.getAttribute("destinationPoint");
      paths.add(new Roadmap.Path(e.getAttribute("name"), src, dst, length, false, false));
      if (bidir) {
        paths.add(
            new Roadmap.Path(e.getAttribute("name") + "~rev", dst, src, length, false, true));
      }
    }
    return new Roadmap.Model(
        points, paths, cargoSlotPoints(doc), dropOffPoints(doc), laneDirectionDeclared(doc));
  }

  private static boolean laneDirectionDeclared(org.w3c.dom.Document doc) {
    NodeList layouts = doc.getElementsByTagName("visualLayout");
    for (int i = 0; i < layouts.getLength(); i++) {
      NodeList properties = ((Element) layouts.item(i)).getElementsByTagName("property");
      for (int j = 0; j < properties.getLength(); j++) {
        if (LaneAxis.LAYOUT_DIRECTION_PROPERTY
            .equals(((Element) properties.item(j)).getAttribute("name"))) {
          return true;
        }
      }
    }
    return false;
  }

  private static Set<String> dropOffPoints(org.w3c.dom.Document doc) {
    Set<String> dropOffTypes = new HashSet<>();
    NodeList types = doc.getElementsByTagName("locationType");
    for (int i = 0; i < types.getLength(); i++) {
      Element type = (Element) types.item(i);
      List<String> allowedOperations = new ArrayList<>();
      NodeList operations = type.getElementsByTagName("allowedOperation");
      for (int j = 0; j < operations.getLength(); j++) {
        allowedOperations.add(((Element) operations.item(j)).getAttribute("name"));
      }
      if (VehicleOperations.isDropOff(allowedOperations)) {
        dropOffTypes.add(type.getAttribute("name"));
      }
    }

    Set<String> slots = new HashSet<>();
    NodeList locations = doc.getElementsByTagName("location");
    for (int i = 0; i < locations.getLength(); i++) {
      Element location = (Element) locations.item(i);
      if (!dropOffTypes.contains(location.getAttribute("type"))) {
        continue;
      }
      NodeList links = location.getElementsByTagName("link");
      for (int j = 0; j < links.getLength(); j++) {
        slots.add(((Element) links.item(j)).getAttribute("point"));
      }
    }
    return slots;
  }

  private static Set<String> cargoSlotPoints(org.w3c.dom.Document doc) {
    Set<String> waitingCargoTypes = new HashSet<>();
    NodeList types = doc.getElementsByTagName("locationType");
    for (int i = 0; i < types.getLength(); i++) {
      Element type = (Element) types.item(i);
      List<String> allowedOperations = new ArrayList<>();
      NodeList operations = type.getElementsByTagName("allowedOperation");
      for (int j = 0; j < operations.getLength(); j++) {
        allowedOperations.add(((Element) operations.item(j)).getAttribute("name"));
      }
      if (VehicleOperations.isPickUp(allowedOperations)) {
        waitingCargoTypes.add(type.getAttribute("name"));
      }
    }

    Set<String> slots = new HashSet<>();
    NodeList locations = doc.getElementsByTagName("location");
    for (int i = 0; i < locations.getLength(); i++) {
      Element location = (Element) locations.item(i);
      if (!waitingCargoTypes.contains(location.getAttribute("type"))) {
        continue;
      }
      NodeList links = location.getElementsByTagName("link");
      for (int j = 0; j < links.getLength(); j++) {
        slots.add(((Element) links.item(j)).getAttribute("point"));
      }
    }
    return slots;
  }

  public static File findMap(String name) {
    File dir = new File("").getAbsoluteFile();
    for (int i = 0; i < 8 && dir != null; i++, dir = dir.getParentFile()) {
      File candidate = new File(dir, "wes/maps/" + name);
      if (candidate.isFile()) {
        return candidate;
      }
    }
    return null;
  }
}
