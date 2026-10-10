package org.opentcs.fms.kernel.integration;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import javax.xml.parsers.DocumentBuilderFactory;
import org.opentcs.data.model.Block;
import org.opentcs.data.model.Location;
import org.opentcs.data.model.LocationType;
import org.opentcs.data.model.Path;
import org.opentcs.data.model.PlantModel;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.Pose;
import org.opentcs.data.model.TCSResourceReference;
import org.opentcs.data.model.Triple;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

public final class PlantModelXmlLoader {

  private PlantModelXmlLoader() {
  }

  public static PlantModel load(File xml) throws Exception {
    var factory = DocumentBuilderFactory.newInstance();
    factory.setNamespaceAware(false);
    factory.setValidating(false);
    var doc = factory.newDocumentBuilder().parse(xml);
    doc.getDocumentElement().normalize();
    String modelName = doc.getDocumentElement().getAttribute("name");

    Map<String, Point> points = new HashMap<>();
    NodeList pts = doc.getElementsByTagName("point");
    for (int i = 0; i < pts.getLength(); i++) {
      Element e = (Element) pts.item(i);
      if (!e.hasAttribute("positionX")) {
        continue;
      }
      String name = e.getAttribute("name");
      points.put(name, new Point(name).withPose(new Pose(
          new Triple(
              (long) Double.parseDouble(e.getAttribute("positionX")),
              (long) Double.parseDouble(e.getAttribute("positionY")),
              e.hasAttribute("positionZ")
                  ? (long) Double.parseDouble(e.getAttribute("positionZ"))
                  : 0L),
          Double.NaN)));
    }

    Map<String, Path> paths = new HashMap<>();
    NodeList pas = doc.getElementsByTagName("path");
    for (int i = 0; i < pas.getLength(); i++) {
      Element e = (Element) pas.item(i);
      Point src = points.get(e.getAttribute("sourcePoint"));
      Point dst = points.get(e.getAttribute("destinationPoint"));
      if (src == null || dst == null) {
        continue;
      }
      String name = e.getAttribute("name");
      paths.put(name, new Path(name, src.getReference(), dst.getReference())
          .withLength(e.hasAttribute("length")
              ? (long) Double.parseDouble(e.getAttribute("length"))
              : 1L)
          .withMaxVelocity(e.hasAttribute("maxVelocity")
              ? (int) Double.parseDouble(e.getAttribute("maxVelocity"))
              : 1000)
          .withMaxReverseVelocity(e.hasAttribute("maxReverseVelocity")
              ? (int) Double.parseDouble(e.getAttribute("maxReverseVelocity"))
              : 0)
          .withLocked("true".equals(e.getAttribute("locked"))));
    }

    Map<String, LocationType> locationTypes = new HashMap<>();
    NodeList lts = doc.getElementsByTagName("locationType");
    for (int i = 0; i < lts.getLength(); i++) {
      Element e = (Element) lts.item(i);
      List<String> operations = new ArrayList<>();
      NodeList ops = e.getElementsByTagName("allowedOperation");
      for (int j = 0; j < ops.getLength(); j++) {
        operations.add(((Element) ops.item(j)).getAttribute("name"));
      }
      String name = e.getAttribute("name");
      locationTypes.put(name, new LocationType(name).withAllowedOperations(operations));
    }

    Map<String, Location> locations = new HashMap<>();
    NodeList locs = doc.getElementsByTagName("location");
    for (int i = 0; i < locs.getLength(); i++) {
      Element e = (Element) locs.item(i);
      LocationType type = locationTypes.get(e.getAttribute("type"));
      if (type == null) {
        continue;
      }
      String name = e.getAttribute("name");
      Location location = new Location(name, type.getReference());
      Set<Location.Link> links = new HashSet<>();
      NodeList lks = e.getElementsByTagName("link");
      for (int j = 0; j < lks.getLength(); j++) {
        Point linked = points.get(((Element) lks.item(j)).getAttribute("point"));
        if (linked != null) {
          links.add(new Location.Link(location.getReference(), linked.getReference()));
        }
      }
      locations.put(name, location.withAttachedLinks(links));
    }

    Set<Block> blocks = new HashSet<>();
    NodeList bls = doc.getElementsByTagName("block");
    for (int i = 0; i < bls.getLength(); i++) {
      Element e = (Element) bls.item(i);
      Set<TCSResourceReference<?>> members = new HashSet<>();
      NodeList ms = e.getElementsByTagName("member");
      for (int j = 0; j < ms.getLength(); j++) {
        String member = ((Element) ms.item(j)).getAttribute("name");
        if (points.containsKey(member)) {
          members.add(points.get(member).getReference());
        } else if (paths.containsKey(member)) {
          members.add(paths.get(member).getReference());
        }
      }
      Block.Type type = "SINGLE_VEHICLE_ONLY".equals(e.getAttribute("type"))
          ? Block.Type.SINGLE_VEHICLE_ONLY
          : Block.Type.SAME_DIRECTION_ONLY;
      blocks.add(new Block(e.getAttribute("name")).withType(type).withMembers(members));
    }

    return new PlantModel(modelName)
        .withPoints(Set.copyOf(points.values()))
        .withPaths(Set.copyOf(paths.values()))
        .withLocationTypes(Set.copyOf(locationTypes.values()))
        .withLocations(Set.copyOf(locations.values()))
        .withBlocks(blocks);
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
