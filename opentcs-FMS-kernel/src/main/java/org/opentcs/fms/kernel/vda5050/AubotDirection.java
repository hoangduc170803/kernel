package org.opentcs.fms.kernel.vda5050;

import javax.annotation.Nullable;
import org.opentcs.data.model.Point;

public enum AubotDirection {

  NORTH("Y+"),
  EAST("X+"),
  SOUTH("Y-"),
  WEST("X-");

  private final String code;

  AubotDirection(String code) {
    this.code = code;
  }

  public String getCode() {
    return code;
  }

  public AubotDirection reverse() {
    return values()[(ordinal() + 2) % 4];
  }

  public Coordinates toVehicleFrame(double mapX, double mapY) {
    return switch (this) {
      case NORTH -> new Coordinates(mapX, mapY);
      case EAST -> new Coordinates(-mapY, mapX);
      case SOUTH -> new Coordinates(-mapX, -mapY);
      case WEST -> new Coordinates(mapY, -mapX);
    };
  }

  public Coordinates toMapFrame(double vehicleX, double vehicleY) {
    return switch (this) {
      case NORTH -> new Coordinates(vehicleX, vehicleY);
      case EAST -> new Coordinates(vehicleY, -vehicleX);
      case SOUTH -> new Coordinates(-vehicleX, -vehicleY);
      case WEST -> new Coordinates(-vehicleY, vehicleX);
    };
  }

  public record Coordinates(double x, double y) {
  }

  public AubotDirection alignTo(@Nullable AubotDirection reference) {
    return reference == null ? this : values()[(ordinal() - reference.ordinal() + 4) % 4];
  }

  @Nullable
  public static AubotDirection fromCode(@Nullable String code) {
    if (code == null) {
      return null;
    }

    for (AubotDirection direction : values()) {
      if (direction.code.equalsIgnoreCase(code.trim())) {
        return direction;
      }
    }
    return null;
  }

  @Nullable
  public static AubotDirection between(Point source, Point destination, boolean backward) {
    long sourceX = source.getPose().getPosition().getX();
    long sourceY = source.getPose().getPosition().getY();
    long destinationX = destination.getPose().getPosition().getX();
    long destinationY = destination.getPose().getPosition().getY();

    if (sourceX == destinationX && sourceY == destinationY) {
      return null;
    }

    AubotDirection direction
        = Math.abs(destinationX - sourceX) > Math.abs(destinationY - sourceY)
            ? (destinationX > sourceX ? EAST : WEST)
            : (destinationY > sourceY ? NORTH : SOUTH);

    return backward ? direction.reverse() : direction;
  }
}
