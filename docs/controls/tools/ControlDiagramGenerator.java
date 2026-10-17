import java.io.IOException;
import java.io.Reader;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.time.LocalDate;
import java.util.ArrayList;
import java.util.Base64;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Properties;
import java.util.Set;
import java.util.TreeMap;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Collectors;
import java.util.stream.Stream;

/**
 * Builds versioned controller diagrams from the ControlBoard implementations.
 *
 * <p>Run from the repo root (JDK 11+, no dependencies):
 *
 * <pre>
 *   java docs/controls/tools/ControlDiagramGenerator.java generate [--note "why"] [--force]
 *   java docs/controls/tools/ControlDiagramGenerator.java check
 *   java docs/controls/tools/ControlDiagramGenerator.java add-image &lt;type&gt; &lt;image file&gt; [--from xbox/v1]
 * </pre>
 *
 * Or through Gradle: {@code ./gradlew controlDiagrams -Pnote="why"}.
 */
public class ControlDiagramGenerator {
  static final String CONTROLS = "docs/controls";
  static final String CONTROL_BOARD =
      "src/main/java/org/jmhsrobotics/frc2026/controlBoard/ControlBoard.java";
  static final String OTHER_COLOR = "#6b7280";
  static final String UNWIRED_COLOR = "#9ca3af";
  static final String CONFLICT_COLOR = "#dc2626";

  // Diagram layout (SVG pixels)
  static final int WIDTH = 1280;
  static final int CTRL_W = 500;
  static final int CTRL_X = (WIDTH - CTRL_W) / 2;
  static final int LINE_H = 17;
  static final int BLOCK_GAP = 10;

  /** Order inputs appear in the bindings tables. */
  static final List<String> INPUT_ORDER =
      List.of(
          "leftStick",
          "rightStick",
          "leftTrigger",
          "rightTrigger",
          "leftBumper",
          "rightBumper",
          "a",
          "b",
          "x",
          "y",
          "povUp",
          "povDown",
          "povLeft",
          "povRight",
          "back",
          "start");

  record Binding(String controller, String function, String input, String detail) {}

  record Fn(String group, String label) {}

  record Anchor(double x, double y, boolean left, String name) {}

  record ControllerImage(String ref, Path dir, Properties props, Map<String, Anchor> anchors) {}

  static Path root;
  static Map<String, Fn> catalog = new LinkedHashMap<>();
  static Map<String, String> groupColors = new LinkedHashMap<>();
  static Set<String> wired;

  public static void main(String[] args) throws Exception {
    root = findRepoRoot();
    String cmd = args.length > 0 ? args[0] : "generate";
    switch (cmd) {
      case "generate", "check" -> generate(cmd.equals("check"), parseFlags(args));
      case "add-image" -> {
        if (args.length < 3) die("usage: add-image <type> <image file> [--from <type>/v<N>]");
        String from = args.length >= 5 && args[3].equals("--from") ? args[4] : null;
        addImage(args[1], Path.of(args[2]), from);
      }
      default -> die("unknown command '" + cmd + "' (generate | check | add-image)");
    }
  }

  // ===================== generate / check =====================

  static void generate(boolean checkOnly, Map<String, String> flags) throws IOException {
    loadCatalog();
    wired = findWiredFunctions();
    List<String> interfaceOrder = parseInterfaceMethods();
    for (String fn : interfaceOrder) {
      if (!catalog.containsKey(fn)) {
        warn(fn + "() is in ControlBoard but not in functions.properties; using the method name");
      }
    }

    Properties configs = load(root.resolve(CONTROLS + "/configs.properties"));
    boolean stale = false;
    for (String cfg : list(configs.getProperty("configs"))) {
      String source = req(configs, cfg + ".source");
      List<String> controllers = list(req(configs, cfg + ".controllers"));
      Map<String, ControllerImage> images = new LinkedHashMap<>();
      for (String c : controllers) {
        images.put(c, loadImage(req(configs, cfg + "." + c + ".image")));
      }

      List<Binding> bindings = parseBindings(root.resolve(source), images);
      for (Binding b : bindings) {
        if (!images.get(b.controller()).anchors().containsKey(b.input())) {
          die(
              source
                  + ": "
                  + b.function()
                  + "() uses "
                  + b.input()
                  + " but "
                  + images.get(b.controller()).ref()
                  + " has no anchor for it");
        }
      }
      Set<String> bound = bindings.stream().map(Binding::function).collect(Collectors.toSet());
      List<String> unbound =
          interfaceOrder.stream().filter(f -> !bound.contains(f)).collect(Collectors.toList());

      String table = bindingsMarkdown(configs, cfg, controllers, images, bindings, unbound);
      Path cfgDir = root.resolve(CONTROLS + "/diagrams/" + cfg);
      int latest = latestVersion(cfgDir);
      boolean changed =
          latest == 0
              || !Files.readString(cfgDir.resolve("v" + latest + "/bindings.md")).equals(table);

      if (checkOnly) {
        if (changed) {
          System.out.println("STALE  " + cfg + ": bindings differ from v" + latest);
          stale = true;
        } else {
          System.out.println("OK     " + cfg + ": matches v" + latest);
        }
        continue;
      }
      if (!changed && !flags.containsKey("force")) {
        System.out.println("unchanged  " + cfg + " (still v" + latest + ")");
        continue;
      }

      int version = latest + 1;
      Path out = cfgDir.resolve("v" + version);
      Files.createDirectories(out);
      String commit = gitCommit();
      String date = LocalDate.now().toString();
      Files.writeString(out.resolve("bindings.md"), table);
      Files.writeString(
          out.resolve("diagram.svg"),
          svg(configs, cfg, controllers, images, bindings, unbound, version, date, commit));
      Properties meta = new Properties();
      meta.setProperty("date", date);
      meta.setProperty("commit", commit);
      meta.setProperty("source", source);
      meta.setProperty("note", flags.getOrDefault("note", ""));
      try (var w = Files.newBufferedWriter(out.resolve("version.properties"))) {
        meta.store(w, "Generated by ControlDiagramGenerator");
      }
      System.out.println("created    " + cfg + " v" + version + " -> " + root.relativize(out));
    }

    if (checkOnly) {
      if (stale) {
        die("controller diagrams are out of date: run ./gradlew controlDiagrams");
      }
      return;
    }
    writeMaster(configs);
  }

  /**
   * Reads `driver.rightBumper()` / `joystick.button(3)` / `pad.pov(90)` calls out of each method
   * body of a ControlBoard impl. How a call maps to a spot on the picture comes from that
   * controller image's anchors.properties, so any controller type works.
   */
  static List<Binding> parseBindings(Path source, Map<String, ControllerImage> images)
      throws IOException {
    String code = stripComments(Files.readString(source));
    Pattern method =
        Pattern.compile(
            "public\\s+\\w+\\s+(\\w+)\\s*\\(\\s*\\)\\s*\\{(.*?)\\n\\s*\\}", Pattern.DOTALL);
    Pattern call =
        Pattern.compile(
            "\\b("
                + String.join("|", images.keySet())
                + ")\\s*\\.\\s*(\\w+)\\s*\\(\\s*([\\w.]*)\\s*\\)");
    List<Binding> out = new ArrayList<>();
    Matcher m = method.matcher(code);
    while (m.find()) {
      Matcher c = call.matcher(m.group(2));
      while (c.find()) {
        // button(3) -> "button3", pov(90) -> "pov90", a() -> "a"
        String key = c.group(2) + c.group(3).replaceAll("\\W", "");
        String[] input = toInput(images.get(c.group(1)), key);
        if (input != null) out.add(new Binding(c.group(1), m.group(1), input[0], input[1]));
      }
    }
    return out;
  }

  /**
   * Controller call -> {anchor key, detail}, or null for calls that are not inputs. Uses the
   * image's `input.<call>=<anchor>,<detail>` aliases and `ignore=` list; otherwise the call name is
   * the anchor key.
   */
  static String[] toInput(ControllerImage img, String call) {
    if (list(img.props().getProperty("ignore", "getHID")).contains(call)) return null;
    String alias = img.props().getProperty("input." + call);
    if (alias == null) return new String[] {call, ""};
    String[] parts = alias.split(",", 2);
    return new String[] {parts[0].trim(), parts.length > 1 ? parts[1].trim() : ""};
  }

  static List<String> parseInterfaceMethods() throws IOException {
    String code = stripComments(Files.readString(root.resolve(CONTROL_BOARD)));
    Matcher m = Pattern.compile("public\\s+\\w+\\s+(\\w+)\\s*\\(\\s*\\)\\s*;").matcher(code);
    List<String> out = new ArrayList<>();
    while (m.find()) out.add(m.group(1));
    return out;
  }

  /** ControlBoard methods actually called somewhere outside the controlBoard package. */
  static Set<String> findWiredFunctions() throws IOException {
    List<String> fns = parseInterfaceMethods();
    Set<String> found = new LinkedHashSet<>();
    try (Stream<Path> files = Files.walk(root.resolve("src/main/java"))) {
      for (Path p :
          files.filter(p -> p.toString().endsWith(".java")).collect(Collectors.toList())) {
        if (p.toString().contains("controlBoard")) continue;
        String code = stripComments(Files.readString(p));
        for (String fn : fns) {
          if (Pattern.compile("\\.\\s*" + fn + "\\s*\\(\\s*\\)").matcher(code).find()) {
            found.add(fn);
          }
        }
      }
    }
    return found;
  }

  // ===================== output: markdown =====================

  static String bindingsMarkdown(
      Properties configs,
      String cfg,
      List<String> controllers,
      Map<String, ControllerImage> images,
      List<Binding> bindings,
      List<String> unbound) {
    StringBuilder sb = new StringBuilder();
    for (String c : controllers) {
      ControllerImage img = images.get(c);
      sb.append("#### ")
          .append(configs.getProperty(cfg + "." + c + ".role"))
          .append(" (port ")
          .append(configs.getProperty(cfg + "." + c + ".port"))
          .append(", `")
          .append(c)
          .append("`, image `")
          .append(img.ref())
          .append("`)\n\n");
      sb.append("| Input | Robot function | Group | Status |\n|---|---|---|---|\n");
      for (Binding b : sorted(bindings, c)) {
        String input =
            img.anchors().get(b.input()).name()
                + (b.detail().isEmpty() ? "" : " (" + b.detail() + ")");
        String status =
            !wired.contains(b.function())
                ? "not wired in robot code"
                : conflicts(bindings, b) ? "**CONFLICT**" : "ok";
        sb.append("| ")
            .append(input)
            .append(" | ")
            .append(fn(b.function()).label())
            .append(" `")
            .append(b.function())
            .append("()` | ")
            .append(fn(b.function()).group())
            .append(" | ")
            .append(status)
            .append(" |\n");
      }
      sb.append("\n");
    }
    if (!unbound.isEmpty()) {
      sb.append("**Not on a button in this config:** ")
          .append(
              unbound.stream()
                  .map(f -> fn(f).label() + " `" + f + "()`")
                  .collect(Collectors.joining(", ")))
          .append("\n");
    }
    return sb.toString();
  }

  static void writeMaster(Properties configs) throws IOException {
    StringBuilder sb = new StringBuilder();
    sb.append("# Controller Diagrams - MASTER\n\n")
        .append("> Generated by `docs/controls/tools/ControlDiagramGenerator.java`. Do not edit by")
        .append(" hand; run `./gradlew controlDiagrams` instead. See [README](README.md).\n\n");
    List<String> cfgs = list(configs.getProperty("configs"));
    for (String cfg : cfgs) {
      String title = configs.getProperty(cfg + ".title");
      sb.append("- [").append(title).append("](#").append(slug(title)).append(")\n");
    }
    for (String cfg : cfgs) {
      Path dir = root.resolve(CONTROLS + "/diagrams/" + cfg);
      int latest = latestVersion(dir);
      if (latest == 0) continue;
      Properties meta = load(dir.resolve("v" + latest + "/version.properties"));
      sb.append("\n## ")
          .append(configs.getProperty(cfg + ".title"))
          .append("\n\n")
          .append("**Current: v")
          .append(latest)
          .append("** (")
          .append(meta.getProperty("date"))
          .append(", commit `")
          .append(meta.getProperty("commit"))
          .append("`) from `")
          .append(meta.getProperty("source"))
          .append("`\n\n")
          .append("![")
          .append(cfg)
          .append(" v")
          .append(latest)
          .append("](diagrams/")
          .append(cfg)
          .append("/v")
          .append(latest)
          .append("/diagram.svg)\n\n")
          .append(Files.readString(dir.resolve("v" + latest + "/bindings.md")))
          .append(
              "\n### Version history\n\n| Version | Date | Commit | Note |\n|---|---|---|---|\n");
      for (int v = latest; v >= 1; v--) {
        Path vd = dir.resolve("v" + v);
        if (!Files.exists(vd.resolve("version.properties"))) continue;
        Properties m = load(vd.resolve("version.properties"));
        sb.append("| [v")
            .append(v)
            .append("](diagrams/")
            .append(cfg)
            .append("/v")
            .append(v)
            .append("/diagram.svg) | ")
            .append(m.getProperty("date"))
            .append(" | `")
            .append(m.getProperty("commit"))
            .append("` | ")
            .append(m.getProperty("note", "").replace("|", "\\|"))
            .append(" |\n");
      }
    }
    Files.writeString(root.resolve(CONTROLS + "/MASTER.md"), sb.toString());
    System.out.println("updated    docs/controls/MASTER.md");
  }

  // ===================== output: svg =====================

  static String svg(
      Properties configs,
      String cfg,
      List<String> controllers,
      Map<String, ControllerImage> images,
      List<Binding> bindings,
      List<String> unbound,
      int version,
      String date,
      String commit)
      throws IOException {
    StringBuilder body = new StringBuilder();
    int y = 96;
    for (String c : controllers) {
      String heading =
          configs.getProperty(cfg + "." + c + ".role")
              + "  \u00b7  port "
              + configs.getProperty(cfg + "." + c + ".port");
      body.append(text(40, y, heading, "20", "700", "#111827", "start", false));
      body.append(
          String.format(
              "<line x1=\"40\" y1=\"%d\" x2=\"%d\" y2=\"%d\" stroke=\"#e5e7eb\""
                  + " stroke-width=\"2\"/>%n",
              y + 10, WIDTH - 40, y + 10));
      y = panel(body, images.get(c), sorted(bindings, c), bindings, y + 34) + 30;
    }

    // legend
    int lx = 40;
    for (var e : groupColors.entrySet()) {
      body.append(
          String.format(
              "<rect x=\"%d\" y=\"%d\" width=\"14\" height=\"14\" rx=\"3\" fill=\"%s\"/>%n",
              lx, y - 12, e.getValue()));
      body.append(text(lx + 20, y, e.getKey(), "14", "400", "#111827", "start", false));
      lx += 40 + e.getKey().length() * 9;
    }
    y += 24;
    body.append(
        text(
            40,
            y,
            "grey italic = not wired in robot code",
            "14",
            "400",
            UNWIRED_COLOR,
            "start",
            true));
    body.append(
        text(
            320,
            y,
            "red = two functions on one input",
            "14",
            "700",
            CONFLICT_COLOR,
            "start",
            false));
    if (!unbound.isEmpty()) {
      y += 28;
      String line =
          "Not on a button: "
              + unbound.stream().map(f -> fn(f).label()).collect(Collectors.joining(", "));
      for (String part : wrap(line, 150)) {
        body.append(text(40, y, part, "14", "400", "#374151", "start", false));
        y += 20;
      }
    }
    int height = y + 30;

    String title = "Warbots 2026  -  " + configs.getProperty(cfg + ".title") + "  -  v" + version;
    String sub =
        "Generated "
            + date
            + " from "
            + Path.of(configs.getProperty(cfg + ".source")).getFileName()
            + " @ "
            + commit;
    return String.format(
            "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"%d\" height=\"%d\" viewBox=\"0 0 %d"
                + " %d\" font-family=\"Helvetica, Arial, sans-serif\">%n"
                + "<rect width=\"100%%\" height=\"100%%\" fill=\"#ffffff\"/>%n",
            WIDTH, height, WIDTH, height)
        + text(40, 44, title, "26", "700", "#111827", "start", false)
        + text(40, 68, sub, "13", "400", "#6b7280", "start", false)
        + body
        + "</svg>\n";
  }

  /** Draws one controller with its callouts. Returns the bottom y of the panel. */
  static int panel(
      StringBuilder sb, ControllerImage img, List<Binding> mine, List<Binding> all, int top)
      throws IOException {
    double scale = CTRL_W / Double.parseDouble(img.props().getProperty("width"));
    int ctrlH = (int) Math.round(Double.parseDouble(img.props().getProperty("height")) * scale);
    sb.append(embedImage(img, CTRL_X, top, CTRL_W, ctrlH));

    // group bindings by input, keep anchor order top-to-bottom per side
    Map<String, List<Binding>> byInput = new LinkedHashMap<>();
    for (Binding b : mine) byInput.computeIfAbsent(b.input(), k -> new ArrayList<>()).add(b);
    int bottom = top + ctrlH;
    for (boolean left : new boolean[] {true, false}) {
      List<String> inputs =
          byInput.keySet().stream()
              .filter(i -> img.anchors().get(i).left() == left)
              .sorted((a, b) -> Double.compare(img.anchors().get(a).y(), img.anchors().get(b).y()))
              .collect(Collectors.toList());
      double cursor = top - 10;
      int textX = left ? CTRL_X - 40 : CTRL_X + CTRL_W + 40;
      int elbowX = left ? CTRL_X - 12 : CTRL_X + CTRL_W + 12;
      String anchor = left ? "end" : "start";
      for (String input : inputs) {
        Anchor a = img.anchors().get(input);
        double ax = CTRL_X + a.x() * scale;
        double ay = top + a.y() * scale;
        List<Binding> bs = byInput.get(input);
        double headerY = Math.max(ay + 5, cursor + LINE_H);
        boolean conflict = bs.stream().anyMatch(b -> conflicts(all, b));

        sb.append(
            String.format(
                "<path d=\"M%.1f %.1f L%d %.1f L%d %.1f\" fill=\"none\" stroke=\"#4b5563\""
                    + " stroke-width=\"1.5\"/>%n<circle cx=\"%.1f\" cy=\"%.1f\" r=\"4\""
                    + " fill=\"#111827\"/>%n",
                ax, ay, elbowX, headerY - 5, textX + (left ? 6 : -6), headerY - 5, ax, ay));
        sb.append(
            text(
                textX,
                headerY,
                a.name() + (conflict ? "  - CONFLICT" : ""),
                "15",
                "700",
                conflict ? CONFLICT_COLOR : "#111827",
                anchor,
                false));
        double ly = headerY;
        for (Binding b : bs) {
          ly += LINE_H;
          boolean isWired = wired.contains(b.function());
          String label =
              (b.detail().isEmpty() ? "" : b.detail() + ": ")
                  + fn(b.function()).label()
                  + (isWired ? "" : " (not wired)");
          String color =
              !isWired
                  ? UNWIRED_COLOR
                  : conflicts(all, b)
                      ? CONFLICT_COLOR
                      : groupColors.getOrDefault(fn(b.function()).group(), OTHER_COLOR);
          sb.append(text(textX, ly, label, "14", "400", color, anchor, !isWired));
        }
        cursor = ly + BLOCK_GAP;
        bottom = (int) Math.max(bottom, cursor);
      }
    }
    return bottom;
  }

  static String embedImage(ControllerImage img, int x, int y, int w, int h) throws IOException {
    Path file = img.dir().resolve(img.props().getProperty("image"));
    String name = file.getFileName().toString().toLowerCase();
    if (name.endsWith(".svg")) {
      String s = Files.readString(file).replaceFirst("<\\?xml[^>]*\\?>", "").trim();
      int end = s.indexOf('>');
      String rootTag = s.substring(0, end).replaceAll("\\s(width|height|x|y)=\"[^\"]*\"", "");
      if (!rootTag.contains("viewBox")) {
        rootTag +=
            String.format(
                " viewBox=\"0 0 %s %s\"",
                img.props().getProperty("width"), img.props().getProperty("height"));
      }
      return rootTag
          + String.format(" x=\"%d\" y=\"%d\" width=\"%d\" height=\"%d\"", x, y, w, h)
          + s.substring(end)
          + "\n";
    }
    String mime = name.endsWith(".png") ? "image/png" : "image/jpeg";
    return String.format(
        "<image x=\"%d\" y=\"%d\" width=\"%d\" height=\"%d\" href=\"data:%s;base64,%s\"/>%n",
        x, y, w, h, mime, Base64.getEncoder().encodeToString(Files.readAllBytes(file)));
  }

  static String text(
      double x,
      double y,
      String s,
      String size,
      String weight,
      String fill,
      String anchor,
      boolean italic) {
    return String.format(
        "<text x=\"%.1f\" y=\"%.1f\" font-size=\"%s\" font-weight=\"%s\" fill=\"%s\""
            + " text-anchor=\"%s\"%s>%s</text>%n",
        x, y, size, weight, fill, anchor, italic ? " font-style=\"italic\"" : "", xml(s));
  }

  // ===================== add-image =====================

  /**
   * Saves a controller picture as controller-images/<type>/v(N+1). Anchors are seeded from `from`
   * (e.g. "xbox/v1"), else the type's previous version, else xbox/v1; width/height are read from
   * the image.
   */
  static void addImage(String type, Path image, String from) throws IOException {
    if (!Files.exists(image)) die("no such file: " + image);
    Path typeDir = root.resolve(CONTROLS + "/controller-images/" + type);
    int latest = latestVersion(typeDir);
    Path out = typeDir.resolve("v" + (latest + 1));
    Files.createDirectories(out);
    String ext = image.getFileName().toString().replaceAll("^.*\\.", "").toLowerCase();
    Path dest = out.resolve("controller." + ext);
    Files.copy(image, dest, StandardCopyOption.REPLACE_EXISTING);

    String seedRef = from != null ? from : latest > 0 ? type + "/v" + latest : "xbox/v1";
    Path seed = root.resolve(CONTROLS + "/controller-images/" + seedRef + "/anchors.properties");
    if (!Files.exists(seed)) die("no anchors to copy at " + root.relativize(seed));
    String anchors =
        Files.readString(seed).replaceAll("(?m)^image=.*$", "image=" + dest.getFileName());
    int[] size = imageSize(dest);
    if (size != null) {
      anchors =
          anchors
              .replaceAll("(?m)^width=.*$", "width=" + size[0])
              .replaceAll("(?m)^height=.*$", "height=" + size[1]);
    }
    Files.writeString(out.resolve("anchors.properties"), anchors);
    System.out.println("saved " + root.relativize(dest) + " (anchors copied from " + seedRef + ")");
    System.out.println(
        "NEXT: edit "
            + root.relativize(out.resolve("anchors.properties"))
            + " so each anchor sits on its button,");
    System.out.println(
        "      then set <config>.<controller>.image="
            + type
            + "/v"
            + (latest + 1)
            + " in configs.properties and run generate.");
  }

  /** {width, height} of a png/jpg, or from an svg's viewBox; null if unknown. */
  static int[] imageSize(Path file) throws IOException {
    if (file.toString().toLowerCase().endsWith(".svg")) {
      Matcher m =
          Pattern.compile("viewBox=\"[\\d.\\s-]*?\\s([\\d.]+)\\s+([\\d.]+)\"")
              .matcher(Files.readString(file));
      return m.find()
          ? new int[] {(int) Double.parseDouble(m.group(1)), (int) Double.parseDouble(m.group(2))}
          : null;
    }
    var img = javax.imageio.ImageIO.read(file.toFile());
    return img == null ? null : new int[] {img.getWidth(), img.getHeight()};
  }

  // ===================== helpers =====================

  static ControllerImage loadImage(String ref) throws IOException {
    Path dir = root.resolve(CONTROLS + "/controller-images/" + ref);
    Properties p = load(dir.resolve("anchors.properties"));
    Map<String, Anchor> anchors = new TreeMap<>();
    for (String k : p.stringPropertyNames()) {
      if (!k.startsWith("anchor.")) continue;
      String[] v = p.getProperty(k).split(",", 4);
      anchors.put(
          k.substring(7),
          new Anchor(
              Double.parseDouble(v[0].trim()),
              Double.parseDouble(v[1].trim()),
              v[2].trim().equals("left"),
              v[3].trim()));
    }
    return new ControllerImage(ref, dir, p, anchors);
  }

  static void loadCatalog() throws IOException {
    Properties p = load(root.resolve(CONTROLS + "/functions.properties"));
    for (String g : list(p.getProperty("groups", ""))) {
      groupColors.put(g, p.getProperty("group." + g, OTHER_COLOR));
    }
    for (String k : p.stringPropertyNames()) {
      if (!k.startsWith("fn.")) continue;
      String[] v = p.getProperty(k).split("\\|", 2);
      catalog.put(k.substring(3), new Fn(v[0].trim(), v.length > 1 ? v[1].trim() : k.substring(3)));
    }
  }

  static Fn fn(String name) {
    return catalog.getOrDefault(name, new Fn("Other", name));
  }

  /** True when another wired function shares this binding's controller + input + axis. */
  static boolean conflicts(List<Binding> all, Binding b) {
    return wired.contains(b.function())
        && all.stream()
            .anyMatch(
                o ->
                    o != b
                        && wired.contains(o.function())
                        && !o.function().equals(b.function())
                        && o.controller().equals(b.controller())
                        && o.input().equals(b.input())
                        && o.detail().equals(b.detail()));
  }

  static List<Binding> sorted(List<Binding> bindings, String controller) {
    return bindings.stream()
        .filter(b -> b.controller().equals(controller))
        .sorted(
            (a, b) -> {
              int c = Integer.compare(order(a.input()), order(b.input()));
              return c != 0 ? c : (a.detail() + a.function()).compareTo(b.detail() + b.function());
            })
        .collect(Collectors.toList());
  }

  static int order(String input) {
    int i = INPUT_ORDER.indexOf(input);
    return i < 0 ? INPUT_ORDER.size() : i;
  }

  static int latestVersion(Path dir) throws IOException {
    if (!Files.isDirectory(dir)) return 0;
    try (Stream<Path> s = Files.list(dir)) {
      return s.map(p -> p.getFileName().toString())
          .filter(n -> n.matches("v\\d+"))
          .mapToInt(n -> Integer.parseInt(n.substring(1)))
          .max()
          .orElse(0);
    }
  }

  static String gitCommit() {
    String hash = run("git", "rev-parse", "--short", "HEAD");
    if (hash.isEmpty()) return "unknown";
    return run("git", "status", "--porcelain", "--", "src/main/java").isEmpty()
        ? hash
        : hash + "+uncommitted";
  }

  static String run(String... cmd) {
    try {
      Process p =
          new ProcessBuilder(cmd).directory(root.toFile()).redirectErrorStream(true).start();
      String out = new String(p.getInputStream().readAllBytes(), StandardCharsets.UTF_8).trim();
      return p.waitFor() == 0 ? out : "";
    } catch (IOException | InterruptedException e) {
      return "";
    }
  }

  static String stripComments(String code) {
    return code.replaceAll("(?s)/\\*.*?\\*/", "").replaceAll("//[^\\n]*", "");
  }

  static Map<String, String> parseFlags(String[] args) {
    Map<String, String> flags = new LinkedHashMap<>();
    for (int i = 1; i < args.length; i++) {
      if (args[i].equals("--force")) flags.put("force", "true");
      else if (args[i].equals("--note") && i + 1 < args.length) flags.put("note", args[++i]);
      else if (args[i].startsWith("--note=")) flags.put("note", args[i].substring(7));
      else die("unknown option " + args[i]);
    }
    return flags;
  }

  static Path findRepoRoot() {
    Path p = Path.of("").toAbsolutePath();
    while (p != null && !Files.exists(p.resolve(CONTROLS + "/configs.properties"))) {
      p = p.getParent();
    }
    if (p == null) die("run this from inside the Warbots2026 repo");
    return p;
  }

  static Properties load(Path file) throws IOException {
    Properties p = new Properties();
    try (Reader r = Files.newBufferedReader(file)) {
      p.load(r);
    }
    return p;
  }

  static String req(Properties p, String key) {
    String v = p.getProperty(key);
    if (v == null) die("configs.properties is missing " + key);
    return v;
  }

  static List<String> list(String csv) {
    List<String> out = new ArrayList<>();
    for (String s : csv.split(",")) if (!s.isBlank()) out.add(s.trim());
    return out;
  }

  static List<String> wrap(String s, int width) {
    List<String> lines = new ArrayList<>();
    StringBuilder line = new StringBuilder();
    for (String word : s.split(" ")) {
      if (line.length() + word.length() + 1 > width && line.length() > 0) {
        lines.add(line.toString());
        line.setLength(0);
      }
      if (line.length() > 0) line.append(' ');
      line.append(word);
    }
    if (line.length() > 0) lines.add(line.toString());
    return lines;
  }

  static String slug(String s) {
    return s.toLowerCase().replaceAll("[^a-z0-9 -]", "").replace(' ', '-');
  }

  static String xml(String s) {
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;");
  }

  static void warn(String msg) {
    System.err.println("WARNING: " + msg);
  }

  static void die(String msg) {
    System.err.println("ERROR: " + msg);
    System.exit(1);
  }
}
