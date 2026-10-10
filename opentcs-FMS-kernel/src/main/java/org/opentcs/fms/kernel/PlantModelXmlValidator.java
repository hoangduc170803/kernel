package org.opentcs.fms.kernel;

import java.io.IOException;
import java.io.StringReader;
import org.opentcs.util.persistence.ProbePlantModelTO;
import org.opentcs.util.persistence.v7.V7ModelParser;
import org.xml.sax.SAXParseException;

/**
 * Checks plant model XML with the exact parser chain the Model Editor uses to open a file
 * ({@code ModelParser.readModel}: version probe, then {@link V7ModelParser} with schema validation
 * and conversion of older model versions), so a file accepted here opens in the Model Editor built
 * on the same openTCS release.
 */
public final class PlantModelXmlValidator {

  public PlantModelXmlValidator() {
  }

  public Result validate(String xml) {
    String version;
    try {
      version = ProbePlantModelTO.fromXml(new StringReader(xml)).getVersion();
    }
    catch (IOException | RuntimeException e) {
      return Result.invalid(null, e);
    }
    if (version == null || version.isBlank()) {
      return new Result(false, null, "The <model> element has no 'version' attribute.", null, null);
    }

    try {
      new V7ModelParser().read(new StringReader(xml), version);
      return new Result(true, version, null, null, null);
    }
    catch (IOException | RuntimeException e) {
      return Result.invalid(version, e);
    }
  }

  public record Result(
      boolean valid,
      String version,
      String message,
      Integer line,
      Integer column
  ) {

    static Result invalid(String version, Throwable error) {
      SAXParseException location = findCause(error, SAXParseException.class);
      if (location != null) {
        return new Result(
            false,
            version,
            location.getMessage(),
            location.getLineNumber(),
            location.getColumnNumber()
        );
      }
      return new Result(false, version, deepestMessage(error), null, null);
    }

    private static <T extends Throwable> T findCause(Throwable error, Class<T> type) {
      for (Throwable current = error; current != null; current = current.getCause()) {
        if (type.isInstance(current)) {
          return type.cast(current);
        }
        if (current.getCause() == current) {
          break;
        }
      }
      return null;
    }

    private static String deepestMessage(Throwable error) {
      String message = error.toString();
      for (Throwable current = error; current != null; current = current.getCause()) {
        if (current.getMessage() != null && !current.getMessage().isBlank()) {
          message = current.getMessage();
        }
        if (current.getCause() == current) {
          break;
        }
      }
      return message;
    }
  }
}
