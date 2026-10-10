package org.opentcs.fms.kernel.transformers;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import org.opentcs.data.model.Path;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.Pose;
import org.opentcs.data.model.Triple;
import org.opentcs.data.model.Vehicle;
import org.opentcs.data.order.Route;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

public final class PlantModelXmlFixture {

  private final Map<String, Point> points;
  private final Map<String, Path> pathsBySourceAndDestination;

  private PlantModelXmlFixture(
      Map<String, Point> points,
      Map<String, Path> pathsBySourceAndDestination
  ) {
    this.points = points;
    this.pathsBySourceAndDestination = pathsBySourceAndDestination;
  }

  public static PlantModelXmlFixture load(File xml)
      throws Exception {
    DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
    factory.setNamespaceAware(false);
    factory.setValidating(false);
    try {
      factory.setFeature("http://apache.org/xml/features/nonvalidating/load-external-dtd", false);
    }
    catch (Exception ignored) {
    }
    DocumentBuilder builder = factory.newDocumentBuilder();
    var document = builder.parse(xml);
    document.getDocumentElement().normalize();

    Map<String, Point> points = new HashMap<>();
    NodeList pointElements = document.getElementsByTagName("point");
    for (int i = 0; i < pointElements.getLength(); i++) {
      Element element = (Element) pointElements.item(i);
      if (!element.hasAttribute("positionX")) {
        continue;
      }
      points.put(element.getAttribute("name"), pointOf(element));
    }

    Map<String, Path> paths = new HashMap<>();
    NodeList pathElements = document.getElementsByTagName("path");
    for (int i = 0; i < pathElements.getLength(); i++) {
      Element element = (Element) pathElements.item(i);
      if (!element.hasAttribute("sourcePoint") || !element.hasAttribute("destinationPoint")) {
        continue;
      }
      String source = element.getAttribute("sourcePoint");
      String destination = element.getAttribute("destinationPoint");
      Point sourcePoint = points.get(source);
      Point destinationPoint = points.get(destination);
      if (sourcePoint == null || destinationPoint == null) {
        continue;
      }
      Path path = new Path(
          element.getAttribute("name"),
          sourcePoint.getReference(),
          destinationPoint.getReference());
      paths.put(key(source, destination), path);
      boolean traversableInReverse = !element.hasAttribute("maxReverseVelocity")
          || Double.parseDouble(element.getAttribute("maxReverseVelocity")) > 0.0;
      if (traversableInReverse) {
        paths.putIfAbsent(key(destination, source), path);
      }
    }

    return new PlantModelXmlFixture(points, paths);
  }

  public Point point(String name) {
    Point point = points.get(name);
    if (point == null) {
      throw new IllegalArgumentException("No point " + name + " in the plant model");
    }
    return point;
  }

  public Route routeThrough(String... pointNames) {
    List<Route.Step> steps = new ArrayList<>();
    for (int i = 1; i < pointNames.length; i++) {
      Point source = point(pointNames[i - 1]);
      Point destination = point(pointNames[i]);
      Path path = pathsBySourceAndDestination.get(key(source.getName(), destination.getName()));
      if (path == null) {
        throw new IllegalArgumentException(
            "No path between " + source.getName() + " and " + destination.getName());
      }
      boolean traversedInReverse = path.getSourcePoint().getName().equals(destination.getName());
      steps.add(
          new Route.Step(
              path,
              source,
              destination,
              traversedInReverse ? Vehicle.Orientation.BACKWARD : Vehicle.Orientation.FORWARD,
              i - 1,
              1));
    }
    return new Route(steps);
  }

  private static Point pointOf(Element element) {
    Point point = new Point(element.getAttribute("name"))
        .withPose(
            new Pose(
                new Triple(
                    Long.parseLong(element.getAttribute("positionX")),
                    Long.parseLong(element.getAttribute("positionY")),
                    element.hasAttribute("positionZ")
                        ? Long.parseLong(element.getAttribute("positionZ"))
                        : 0L),
                Double.NaN))
        .withType(
            "PARK_POSITION".equals(element.getAttribute("type"))
                ? Point.Type.PARK_POSITION
                : Point.Type.HALT_POSITION);

    NodeList properties = element.getElementsByTagName("property");
    for (int i = 0; i < properties.getLength(); i++) {
      Element property = (Element) properties.item(i);
      point = point.withProperty(property.getAttribute("name"), property.getAttribute("value"));
    }
    return point;
  }

  private static String key(String source, String destination) {
    return source + " " + destination;
  }
}
