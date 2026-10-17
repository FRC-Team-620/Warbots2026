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
 *   java docs/controls/tools/ControlDiagramGenerator.java add-source &lt;type&gt; &lt;original image&gt;
 *   java docs/controls/tools/ControlDiagramGenerator.java add-layout &lt;type&gt; &lt;image&gt; [--from xbox/v1]
 *   java docs/controls/tools/ControlDiagramGenerator.java preview-layout &lt;type&gt;/v&lt;N&gt;
 *   java docs/controls/tools/ControlDiagramGenerator.java review
 *   java docs/controls/tools/ControlDiagramGenerator.java sign-off &lt;config&gt; --by "Name" [--note "..."]
 * </pre>
 *
 * Or through Gradle: {@code ./gradlew controlDiagrams -Pnote="why"}.
 */
public class ControlDiagramGenerator {
  static final String CONTROLS = "docs/controls";
  static final String SOURCES = CONTROLS + "/controller-sources";
  static final String LAYOUTS = CONTROLS + "/controller-layouts";

  /** Keys allowed in a layout's anchors.properties: button positions only, never keybindings. */
  static final Pattern LAYOUT_KEY =
      Pattern.compile("name|image|width|height|ignore|source|anchor\\.\\w+|input\\.\\w+");

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
      case "add-source" -> {
        if (args.length < 3) die("usage: add-source <type> <original image>");
        addSource(args[1], Path.of(args[2]));
      }
      case "add-layout" -> {
        if (args.length < 3) die("usage: add-layout <type> <image> [--from <type>/v<N>]");
        String from = args.length >= 5 && args[3].equals("--from") ? args[4] : null;
        addLayout(args[1], Path.of(args[2]), from);
      }
      case "preview-layout" -> {
        if (args.length < 2) die("usage: preview-layout <type>/v<N>");
        previewLayout(args[1]);
      }
      case "review" -> review();
      case "sign-off" -> {
        if (args.length < 2) die("usage: sign-off <config> --by \"Name\" [--note \"...\"]");
        String[] rest = new String[args.length - 1];
        rest[0] = "sign-off";
        System.arraycopy(args, 2, rest, 1, args.length - 2);
        signOff(args[1], parseFlags(rest));
      }
      default ->
          die(
              "unknown command '"
                  + cmd
                  + "' (generate | check | add-source | add-layout | preview-layout | review"
                  + " | sign-off)");
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
    for (String ref : listLayouts()) {
      Path sheet = root.resolve(LAYOUTS + "/" + ref + "/layout.svg");
      String svg = layoutSheet(ref);
      boolean current = Files.exists(sheet) && Files.readString(sheet).equals(svg);
      if (checkOnly) {
        if (!current) {
          System.out.println("STALE  layout " + ref + ": layout.svg out of date");
          stale = true;
        }
      } else if (!current) {
        Files.writeString(sheet, svg);
        System.out.println("updated    " + root.relativize(sheet));
      }
    }
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
    return sb.toString().stripTrailing() + "\n";
  }

  static String humanCheck(Properties meta) {
    String by = meta.getProperty("reviewed.by");
    if (by == null) return "**not yet checked**";
    String note = meta.getProperty("reviewed.note", "");
    return "checked by "
        + by
        + " ("
        + meta.getProperty("reviewed.date")
        + ")"
        + (note.isEmpty() ? "" : ": " + note);
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
          .append("Human check: ")
          .append(humanCheck(meta))
          .append("\n\n")
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
              "\n### Version history\n\n| Version | Date | Commit | Note | Human check |\n"
                  + "|---|---|---|---|---|\n");
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
            .append(" | ")
            .append(humanCheck(m).replace("|", "\\|"))
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
        boolean conflict = bs.stream().anyMatch(b -> b.function() != null && conflicts(all, b));

        // white halo under the leader + dot keeps them readable on busy controller photos
        String leader =
            String.format(
                "M%.1f %.1f L%d %.1f L%d %.1f",
                ax, ay, elbowX, headerY - 5, textX + (left ? 6 : -6), headerY - 5);
        sb.append(
            String.format(
                "<path d=\"%s\" fill=\"none\" stroke=\"#ffffff\" stroke-width=\"4.5\""
                    + " stroke-opacity=\"0.85\"/>%n<path d=\"%s\" fill=\"none\" stroke=\"#374151\""
                    + " stroke-width=\"1.5\"/>%n<circle cx=\"%.1f\" cy=\"%.1f\" r=\"4.5\""
                    + " fill=\"#111827\" stroke=\"#ffffff\" stroke-width=\"2\"/>%n",
                leader, leader, ax, ay));
        boolean oneLine = bs.size() == 1 && bs.get(0).function() == null;
        if (oneLine) { // layout sheet: "LB  leftBumper()" on one line
          sb.append(
              String.format(
                  "<text x=\"%d\" y=\"%.1f\" font-size=\"15\" text-anchor=\"%s\"><tspan"
                      + " font-weight=\"700\" fill=\"#111827\">%s</tspan><tspan fill=\"#4b5563\">"
                      + "  %s</tspan></text>%n",
                  textX, headerY, anchor, xml(a.name()), xml(bs.get(0).detail())));
          bs = List.of();
        } else {
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
        }
        double ly = headerY;
        for (Binding b : bs) {
          ly += LINE_H;
          if (b.function() == null) { // layout sheet: show the Java call, no robot function
            sb.append(text(textX, ly, b.detail(), "14", "400", "#4b5563", anchor, false));
            continue;
          }
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

  // ===================== controller pictures =====================

  /** Archives an original controller picture, untouched, in controller-sources/<type>/. */
  static void addSource(String type, Path image) throws IOException {
    if (!Files.exists(image)) die("no such file: " + image);
    Path dest = root.resolve(SOURCES + "/" + type + "/" + image.getFileName());
    if (Files.exists(dest))
      die(root.relativize(dest) + " already exists; sources are never replaced");
    Files.createDirectories(dest.getParent());
    Files.copy(image, dest);
    System.out.println("saved source " + root.relativize(dest));
    System.out.println(
        "NEXT: crop/clean a copy if needed, then: add-layout " + type + " <image> --from xbox/v1");
  }

  /**
   * Saves a blank controller picture as controller-layouts/<type>/v(N+1). Anchors are seeded from
   * `from` (e.g. "xbox/v1"), else the type's previous version, else xbox/v1; width/height are read
   * from the image.
   */
  static void addLayout(String type, Path image, String from) throws IOException {
    if (!Files.exists(image)) die("no such file: " + image);
    Path typeDir = root.resolve(LAYOUTS + "/" + type);
    int latest = latestVersion(typeDir);
    Path out = typeDir.resolve("v" + (latest + 1));
    Files.createDirectories(out);
    String ext = image.getFileName().toString().replaceAll("^.*\\.", "").toLowerCase();
    Path dest = out.resolve("controller." + ext);
    Files.copy(image, dest, StandardCopyOption.REPLACE_EXISTING);

    String seedRef = from != null ? from : latest > 0 ? type + "/v" + latest : "xbox/v1";
    Path seed = root.resolve(LAYOUTS + "/" + seedRef + "/anchors.properties");
    if (!Files.exists(seed)) die("no anchors to copy at " + root.relativize(seed));
    String anchors =
        Files.readString(seed)
            .replaceAll("(?m)^image=.*$", "image=" + dest.getFileName())
            .replaceAll("(?m)^source=.*\\n?", "");
    int[] size = imageSize(dest);
    if (size != null) {
      anchors =
          anchors
              .replaceAll("(?m)^width=.*$", "width=" + size[0])
              .replaceAll("(?m)^height=.*$", "height=" + size[1]);
    }
    Files.writeString(out.resolve("anchors.properties"), anchors);
    String ref = type + "/v" + (latest + 1);
    System.out.println(
        "saved layout " + root.relativize(dest) + " (anchors copied from " + seedRef + ")");
    System.out.println(
        "NEXT: set source= to the original in controller-sources, move each anchor onto its"
            + " button");
    System.out.println("      (check with: preview-layout " + ref + "), then use image=" + ref);
  }

  /**
   * The committed reference picture for a layout (layout.svg): every button named, with the Java
   * call that reaches it. No keybindings, no dates, so it only changes when the layout does.
   */
  static String layoutSheet(String ref) throws IOException {
    ControllerImage img = loadImage(ref);
    Map<String, List<String>> calls = new TreeMap<>();
    for (String k : img.props().stringPropertyNames()) {
      if (!k.startsWith("input.")) continue;
      String[] v = img.props().getProperty(k).split(",", 2);
      String detail = v.length > 1 && !v[1].isBlank() ? "  (" + v[1].trim() + ")" : "";
      calls
          .computeIfAbsent(v[0].trim(), x -> new ArrayList<>())
          .add(javaCall(k.substring(6)) + detail);
    }
    List<Binding> entries = new ArrayList<>();
    for (String key : img.anchors().keySet()) {
      List<String> cs = calls.getOrDefault(key, List.of(javaCall(key)));
      for (String c : cs.stream().sorted().collect(Collectors.toList())) {
        entries.add(new Binding("layout", null, key, c));
      }
    }
    StringBuilder body = new StringBuilder();
    int y = panel(body, img, entries, entries, 110) + 30;
    body.append(
        text(
            40,
            y,
            "Grey = the Java call that reads this input. Keybindings live in the"
                + " ControlBoard classes, not here.",
            "14",
            "400",
            "#4b5563",
            "start",
            false));
    int height = y + 30;
    return String.format(
            "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"%d\" height=\"%d\" viewBox=\"0 0 %d"
                + " %d\" font-family=\"Helvetica, Arial, sans-serif\">%n"
                + "<rect width=\"100%%\" height=\"100%%\" fill=\"#ffffff\"/>%n",
            WIDTH, height, WIDTH, height)
        + text(40, 44, img.props().getProperty("name", ref), "26", "700", "#111827", "start", false)
        + text(
            40,
            70,
            "Controller layout " + ref + "  \u00b7  button map only, no keybindings",
            "14",
            "400",
            "#6b7280",
            "start",
            false)
        + text(
            40,
            90,
            "Picture: " + img.props().getProperty("source", "(no source= set)"),
            "13",
            "400",
            "#6b7280",
            "start",
            false)
        + body
        + "</svg>\n";
  }

  /** Anchor/alias key back to the Java call: a -> a(), button3 -> button(3). */
  static String javaCall(String key) {
    Matcher m = Pattern.compile("^(\\D+?)(\\d+)$").matcher(key);
    return m.matches() ? m.group(1) + "(" + m.group(2) + ")" : key + "()";
  }

  /** Draws a layout with every anchor and a coordinate grid, no keybindings. */
  static Path previewLayout(String ref) throws IOException {
    ControllerImage img = loadImage(ref);
    int w = (int) Double.parseDouble(img.props().getProperty("width"));
    int h = (int) Double.parseDouble(img.props().getProperty("height"));
    StringBuilder sb = new StringBuilder();
    sb.append(
        String.format(
            "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"%d\" height=\"%d\" viewBox=\"0 0 %d"
                + " %d\" font-family=\"Helvetica, Arial, sans-serif\">%n"
                + "<rect width=\"100%%\" height=\"100%%\" fill=\"#ffffff\"/>%n",
            w, h, w, h));
    sb.append(embedImage(img, 0, 0, w, h));
    int step = Math.max(w, h) > 800 ? 50 : 25;
    for (int x = step; x < w; x += step) {
      sb.append(gridLine(x, 0, x, h, x % (step * 4) == 0));
      if (x % (step * 2) == 0)
        sb.append(text(x + 2, 12, "" + x, "11", "400", "#2563eb", "start", false));
    }
    for (int y = step; y < h; y += step) {
      sb.append(gridLine(0, y, w, y, y % (step * 4) == 0));
      if (y % (step * 2) == 0)
        sb.append(text(2, y - 2, "" + y, "11", "400", "#2563eb", "start", false));
    }
    for (var e : img.anchors().entrySet()) {
      Anchor a = e.getValue();
      sb.append(
          String.format(
              "<circle cx=\"%.1f\" cy=\"%.1f\" r=\"6\" fill=\"#dc2626\" stroke=\"#ffffff\""
                  + " stroke-width=\"2\"/>%n",
              a.x(), a.y()));
      double tx = a.left() ? a.x() - 10 : a.x() + 10;
      sb.append(
          String.format(
              "<text x=\"%.1f\" y=\"%.1f\" font-size=\"14\" font-weight=\"700\" fill=\"#111827\""
                  + " stroke=\"#ffffff\" stroke-width=\"3\" paint-order=\"stroke\""
                  + " text-anchor=\"%s\">%s</text>%n",
              tx, a.y() + 5, a.left() ? "end" : "start", xml(e.getKey())));
    }
    sb.append("</svg>\n");
    Path out = root.resolve("build/control-layouts/" + ref.replace('/', '-') + ".svg");
    Files.createDirectories(out.getParent());
    Files.writeString(out, sb.toString());
    System.out.println("preview " + root.relativize(out) + " (open in a browser)");
    return out;
  }

  static String gridLine(int x1, int y1, int x2, int y2, boolean major) {
    return String.format(
        "<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" stroke=\"#2563eb\" stroke-opacity=\"%s\""
            + " stroke-width=\"1\"/>%n",
        x1, y1, x2, y2, major ? "0.45" : "0.18");
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

  // ===================== human checks =====================

  static final List<String> DIAGRAM_CHECKS =
      List.of(
          "Every label says what the button really does on the robot (ask the drive team)",
          "No text overlaps, is cut off, or points at the wrong button",
          "Every red CONFLICT is fixed, or intentional and explained in the version note",
          "Grey 'not wired' items are expected (e.g. removed mechanisms)",
          "Nothing in 'Not on a button' is something the drivers need in a match");
  static final List<String> LAYOUT_CHECKS =
      List.of(
          "Every red dot sits on its button",
          "Each name is on the correct side (left column vs right column)",
          "Picture is cropped to the controller and easy to read",
          "Dot names match the controller class the Java uses (Xbox: a, leftBumper, ...)");
  static final List<String> ROBOT_CHECKS =
      List.of(
          "Driver Station USB tab: driver on port 0, operator on port 1 (lock them in order)",
          "Robot enabled on blocks or in sim: press every labeled button, it does what the"
              + " diagram says",
          "Sticks: pushing forward drives forward, left drives left (field-relative)",
          "Unlabeled buttons do nothing",
          "The printed diagram on the driver station cart is the current version in MASTER.md");

  /** Builds build/control-review/index.html: current diagrams, layout previews, checklists. */
  static void review() throws IOException {
    Path out = root.resolve("build/control-review/index.html");
    Files.createDirectories(out.getParent());
    Properties configs = load(root.resolve(CONTROLS + "/configs.properties"));
    StringBuilder sb = new StringBuilder();
    sb.append("<!doctype html><html><head><meta charset=\"utf-8\"><title>Controller Review</title>")
        .append("<style>body{font-family:Helvetica,Arial,sans-serif;max-width:1320px;margin:24px")
        .append(" auto;padding:0 16px;color:#111827;background:#fff}img{max-width:100%;border:1px")
        .append(" solid #e5e7eb;border-radius:8px}section{margin:32px 0;padding-top:8px;")
        .append("border-top:2px solid #e5e7eb}label{display:block;margin:6px 0}code{background:")
        .append("#f3f4f6;padding:2px 5px;border-radius:4px}.todo{color:#b45309;font-weight:700}")
        .append(".done{color:#15803d;font-weight:700}</style></head><body>\n")
        .append("<h1>Controller Diagram Review</h1><p>Generated ")
        .append(LocalDate.now())
        .append(". Tick every box, then record it with the <code>sign-off</code> command shown")
        .append(" under each diagram. The boxes here are not saved.</p>\n");

    for (String cfg : list(configs.getProperty("configs"))) {
      Path dir = root.resolve(CONTROLS + "/diagrams/" + cfg);
      int v = latestVersion(dir);
      sb.append("<section><h2>").append(xml(configs.getProperty(cfg + ".title")));
      if (v == 0) {
        sb.append("</h2><p>No diagram yet: run generate.</p></section>\n");
        continue;
      }
      Properties meta = load(dir.resolve("v" + v + "/version.properties"));
      boolean checked = meta.getProperty("reviewed.by") != null;
      sb.append(" - v")
          .append(v)
          .append("</h2><p class=\"")
          .append(checked ? "done" : "todo")
          .append("\">")
          .append(xml(humanCheck(meta).replace("**", "")))
          .append("</p>")
          .append("<img src=\"")
          .append(rel(out, dir.resolve("v" + v + "/diagram.svg")))
          .append("\">")
          .append(checklist(DIAGRAM_CHECKS))
          .append("<p>Done? <code>java docs/controls/tools/ControlDiagramGenerator.java sign-off ")
          .append(cfg)
          .append(" --by \"Your Name\"</code></p></section>\n");
    }

    sb.append("<section><h2>Controller layouts</h2><p>Blank pictures with button positions.")
        .append(" Check these when a layout is added or changed.</p>\n");
    for (String ref : listLayouts()) {
      Path preview = previewLayout(ref);
      sb.append("<h3>")
          .append(ref)
          .append("</h3><img src=\"")
          .append(rel(out, preview))
          .append("\">")
          .append(checklist(LAYOUT_CHECKS));
    }
    sb.append("</section>\n<section><h2>On the robot</h2><p>Do this with the real")
        .append(" controllers before an event and after any binding change.</p>")
        .append(checklist(ROBOT_CHECKS))
        .append("</section></body></html>\n");
    Files.writeString(out, sb.toString());
    System.out.println("review page " + root.relativize(out) + " (open in a browser)");
  }

  static String checklist(List<String> items) {
    return items.stream()
        .map(i -> "<label><input type=\"checkbox\"> " + xml(i) + "</label>")
        .collect(Collectors.joining("", "<div>", "</div>"));
  }

  static String rel(Path page, Path target) {
    return page.getParent().relativize(target).toString().replace('\\', '/');
  }

  static List<String> listLayouts() throws IOException {
    Path layouts = root.resolve(LAYOUTS);
    try (Stream<Path> s = Files.walk(layouts)) {
      return s.filter(p -> Files.exists(p.resolve("anchors.properties")))
          .map(p -> layouts.relativize(p).toString().replace('\\', '/'))
          .sorted()
          .collect(Collectors.toList());
    }
  }

  /** Records that a person checked the current version of a config, then updates MASTER.md. */
  static void signOff(String cfg, Map<String, String> flags) throws IOException {
    String by = flags.get("by");
    if (by == null || by.isBlank()) die("say who checked it: --by \"Your Name\"");
    Path dir = root.resolve(CONTROLS + "/diagrams/" + cfg);
    int v = latestVersion(dir);
    if (v == 0) die("no diagrams for config '" + cfg + "'");
    Path file = dir.resolve("v" + v + "/version.properties");
    Properties meta = load(file);
    meta.setProperty("reviewed.by", by.trim());
    meta.setProperty("reviewed.date", LocalDate.now().toString());
    meta.setProperty("reviewed.note", flags.getOrDefault("note", ""));
    try (var w = Files.newBufferedWriter(file)) {
      meta.store(w, "Generated by ControlDiagramGenerator");
    }
    System.out.println("signed off " + cfg + " v" + v + " by " + by.trim());
    writeMaster(load(root.resolve(CONTROLS + "/configs.properties")));
  }

  // ===================== helpers =====================

  static ControllerImage loadImage(String ref) throws IOException {
    Path dir = root.resolve(LAYOUTS + "/" + ref);
    if (!Files.exists(dir.resolve("anchors.properties"))) {
      die("no layout at " + root.relativize(dir) + " (expected anchors.properties)");
    }
    Properties p = load(dir.resolve("anchors.properties"));
    for (String k : p.stringPropertyNames()) {
      if (!LAYOUT_KEY.matcher(k).matches()) {
        die(
            root.relativize(dir)
                + "/anchors.properties has '"
                + k
                + "': layouts hold button positions only. Keybindings belong in the ControlBoard"
                + " classes, labels in functions.properties.");
      }
    }
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
      else if (args[i].equals("--by") && i + 1 < args.length) flags.put("by", args[++i]);
      else if (args[i].startsWith("--by=")) flags.put("by", args[i].substring(5));
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
