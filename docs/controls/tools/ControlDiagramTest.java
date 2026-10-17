import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Properties;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import javax.xml.parsers.DocumentBuilderFactory;

/**
 * Automated checks for the controller diagram system. Run from the repo root (JDK 11+):
 *
 * <pre>
 *   java docs/controls/tools/ControlDiagramTest.java
 * </pre>
 *
 * Or {@code ./gradlew testControlDiagrams}. Part 1 checks the real repo files; part 2 runs the
 * generator in a throwaway sandbox repo so nothing real is touched. Exits 1 if anything fails.
 */
public class ControlDiagramTest {
  static final String CONTROLS = "docs/controls";
  static final String BOARD_DIR = "src/main/java/org/jmhsrobotics/frc2026/controlBoard";
  static final String MASTER = "MASTER_CONTROLLER-LAYOUT.md";

  record Result(int exit, String output) {}

  interface Check {
    void run() throws Exception;
  }

  static Path repo;
  static Path generator;
  static int passed = 0;
  static List<String> failures = new ArrayList<>();
  static List<Path> sandboxes = new ArrayList<>();

  public static void main(String[] args) throws Exception {
    repo = Path.of("").toAbsolutePath();
    while (repo != null && !Files.exists(repo.resolve(CONTROLS + "/configs.properties"))) {
      repo = repo.getParent();
    }
    if (repo == null) {
      System.err.println("Run this from inside the Warbots2026 repo");
      System.exit(1);
    }
    generator = repo.resolve(CONTROLS + "/tools/ControlDiagramGenerator.java");

    System.out.println("== Real repo ==");
    test("diagrams match the ControlBoard code (check)", ControlDiagramTest::realCheck);
    test("every controller layout is valid", ControlDiagramTest::realLayouts);
    test("every diagram SVG is well-formed XML", ControlDiagramTest::realSvgs);
    test("MASTER links point at real files", ControlDiagramTest::realMasterLinks);

    System.out.println("\n== Generator behavior (sandbox) ==");
    test("first run creates v1, rerun creates nothing", ControlDiagramTest::versioning);
    test("binding change is caught by check and makes v2", ControlDiagramTest::bindingChange);
    test("conflicts and unbound functions are flagged", ControlDiagramTest::flags);
    test("diagrams ignore every file except the keybindings", ControlDiagramTest::onlyKeybindings);
    test("layouts with keybindings are rejected", ControlDiagramTest::layoutGuard);
    test("a button with no anchor is an error", ControlDiagramTest::missingAnchor);
    test("non-Xbox controllers (button(3), pov(90)) work", ControlDiagramTest::joystick);
    test("add-source never overwrites an original", ControlDiagramTest::addSource);
    test("add-layout makes the next version with image size", ControlDiagramTest::addLayout);
    test("generated SVG is well-formed XML", ControlDiagramTest::sandboxSvg);
    test("sign-off is recorded; a new version needs a new check", ControlDiagramTest::signOff);
    test("review page lists diagrams, layouts and checklists", ControlDiagramTest::reviewPage);

    for (Path sb : sandboxes) deleteTree(sb);
    System.out.println();
    if (failures.isEmpty()) {
      System.out.println("ALL " + passed + " CHECKS PASSED");
    } else {
      System.out.println(passed + " passed, " + failures.size() + " FAILED:");
      failures.forEach(f -> System.out.println("  - " + f));
      System.exit(1);
    }
  }

  static void test(String name, Check check) {
    try {
      check.run();
      passed++;
      System.out.println("  PASS  " + name);
    } catch (Throwable t) {
      failures.add(name + ": " + t.getMessage());
      System.out.println(
          "  FAIL  " + name + "\n        " + t.getMessage().replace("\n", "\n        "));
    }
  }

  // ===================== real repo =====================

  static void realCheck() throws Exception {
    Result r = gen(repo, "check");
    expect(r.exit() == 0, "diagrams are out of date, run ./gradlew controlDiagrams\n" + r.output());
  }

  static void realLayouts() throws Exception {
    Path layouts = repo.resolve(CONTROLS + "/controller-layouts");
    List<Path> dirs;
    try (Stream<Path> s = Files.walk(layouts)) {
      dirs =
          s.filter(p -> p.getFileName().toString().matches("v\\d+") && Files.isDirectory(p))
              .sorted()
              .collect(Collectors.toList());
    }
    expect(!dirs.isEmpty(), "no layouts found in " + layouts);
    for (Path dir : dirs) {
      String ref = layouts.relativize(dir).toString().replace('\\', '/');
      Properties p = load(dir.resolve("anchors.properties"));
      expect(Files.exists(dir.resolve(p.getProperty("image", "?"))), ref + ": image= file missing");
      String source = p.getProperty("source");
      expect(source != null, ref + ": no source= line (path to the original picture)");
      expect(
          Files.exists(repo.resolve(CONTROLS).resolve(source)),
          ref + ": source= " + source + " does not exist");
      expect(
          source.startsWith("controller-sources/"),
          ref + ": source= must point into controller-sources/");
      double w = Double.parseDouble(p.getProperty("width"));
      double h = Double.parseDouble(p.getProperty("height"));
      for (String k : p.stringPropertyNames()) {
        if (!k.startsWith("anchor.")) continue;
        String[] v = p.getProperty(k).split(",");
        double x = Double.parseDouble(v[0].trim());
        double y = Double.parseDouble(v[1].trim());
        expect(x >= 0 && x <= w && y >= 0 && y <= h, ref + ": " + k + " is outside the picture");
        expect(v[2].trim().matches("left|right"), ref + ": " + k + " side must be left or right");
      }
      expect(Files.exists(dir.resolve("layout.svg")), ref + ": no layout.svg, run generate");
      parseXml(dir.resolve("layout.svg"));
      Result r = gen(repo, "preview-layout", ref);
      expect(r.exit() == 0, ref + ": " + r.output());
    }
  }

  static void realSvgs() throws Exception {
    try (Stream<Path> s = Files.walk(repo.resolve(CONTROLS + "/diagrams"))) {
      for (Path svg : s.filter(p -> p.toString().endsWith(".svg")).collect(Collectors.toList())) {
        parseXml(svg);
      }
    }
  }

  static void realMasterLinks() throws Exception {
    Path master = repo.resolve(CONTROLS + "/" + MASTER);
    Matcher m = Pattern.compile("\\]\\((diagrams/[^)]+)\\)").matcher(Files.readString(master));
    int links = 0;
    while (m.find()) {
      links++;
      expect(Files.exists(master.getParent().resolve(m.group(1))), "broken link " + m.group(1));
    }
    expect(links > 0, MASTER + " has no diagram links");
  }

  // ===================== sandbox =====================

  static void versioning() throws Exception {
    Path sb = sandbox();
    expectOk(gen(sb, "generate"));
    expect(Files.exists(sb.resolve(CONTROLS + "/diagrams/test/v1/diagram.svg")), "no v1 diagram");
    String sheet =
        Files.readString(sb.resolve(CONTROLS + "/controller-layouts/xbox/v1/layout.svg"));
    expect(sheet.contains("leftBumper()"), "layout.svg missing button calls");
    expect(!sheet.contains("Intake on"), "layout.svg must not contain keybindings");
    expect(Files.exists(sb.resolve(CONTROLS + "/" + MASTER)), "no " + MASTER);
    Result again = gen(sb, "generate");
    expectOk(again);
    expect(again.output().contains("unchanged"), "rerun should report unchanged");
    expect(!Files.exists(sb.resolve(CONTROLS + "/diagrams/test/v2")), "rerun created v2");
    expectOk(gen(sb, "check"));
  }

  static void bindingChange() throws Exception {
    Path sb = sandbox();
    expectOk(gen(sb, "generate"));
    Path impl = sb.resolve(BOARD_DIR + "/TestControl.java");
    write(impl, Files.readString(impl).replace("return driver.b();", "return driver.y();"));
    Result check = gen(sb, "check");
    expect(check.exit() != 0, "check should fail after a binding change");
    expect(check.output().contains("STALE"), "check should say STALE");
    expectOk(gen(sb, "generate", "--note", "moved shoot to Y"));
    expect(
        read(sb, "diagrams/test/v1/bindings.md").contains("| B |"), "v1 must keep the old binding");
    expect(read(sb, "diagrams/test/v2/bindings.md").contains("| Y |"), "v2 missing new binding");
    String master = read(sb, MASTER);
    expect(master.contains("**Current: v2**"), "MASTER should show v2 as current");
    expect(master.contains("moved shoot to Y"), "MASTER history missing the note");
    expect(master.contains("diagrams/test/v1/diagram.svg"), "MASTER history missing v1");
  }

  static void flags() throws Exception {
    Path sb = sandbox();
    expectOk(gen(sb, "generate"));
    String md = read(sb, "diagrams/test/v1/bindings.md");
    expect(row(md, "A", "Intake on").endsWith("| **CONFLICT** |"), "intakeOn on A not a CONFLICT");
    expect(row(md, "A", "Outtake").endsWith("| **CONFLICT** |"), "outtake on A not a CONFLICT");
    expect(row(md, "B", "Shoot").endsWith("| ok |"), "shoot on B should be ok");
    expect(row(md, "X", "Climb").endsWith("| ok |"), "climb should be ok in the diagram");
    expect(!md.contains("not wired"), "diagrams must not depend on other robot code");
    expect(md.contains("**Not on a button in this config:** Turbo"), "turbo should be unbound");
  }

  static void onlyKeybindings() throws Exception {
    Path sb = sandbox();
    expectOk(gen(sb, "generate"));
    Path rc = sb.resolve("src/main/java/org/jmhsrobotics/frc2026/RobotContainer.java");
    write(rc, "class RobotContainer {}\n"); // every function now unused by robot code
    write(
        sb.resolve("src/main/java/Other.java"), "class Other { void x() { control.shoot(); } }\n");
    expectOk(gen(sb, "check"));
    Result again = gen(sb, "generate");
    expectOk(again);
    expect(
        !Files.exists(sb.resolve(CONTROLS + "/diagrams/test/v2")), "non-keybinding edit made v2");
  }

  static void layoutGuard() throws Exception {
    Path sb = sandbox();
    Path anchors = sb.resolve(CONTROLS + "/controller-layouts/xbox/v1/anchors.properties");
    write(anchors, Files.readString(anchors) + "\nfn.intakeOn=Intake|Intake on\n");
    Result r = gen(sb, "generate");
    expect(r.exit() != 0, "generate should refuse a layout with a keybinding");
    expect(r.output().contains("button positions only"), "unexpected error: " + r.output());
  }

  static void missingAnchor() throws Exception {
    Path sb = sandbox();
    Path impl = sb.resolve(BOARD_DIR + "/TestControl.java");
    write(impl, Files.readString(impl).replace("return driver.b();", "return driver.guide();"));
    Result r = gen(sb, "generate");
    expect(r.exit() != 0, "a button with no anchor should fail");
    expect(r.output().contains("has no anchor for it"), "unexpected error: " + r.output());
  }

  static void joystick() throws Exception {
    Path sb = sandbox();
    Path layout = sb.resolve(CONTROLS + "/controller-layouts/stick/v1");
    Files.createDirectories(layout);
    Files.copy(
        sb.resolve(CONTROLS + "/controller-layouts/xbox/v1/controller.svg"),
        layout.resolve("controller.svg"));
    write(
        layout.resolve("anchors.properties"),
        "image=controller.svg\nsource=controller-sources/xbox/xbox-drawing.svg\nwidth=500\n"
            + "height=340\ninput.getX=stick,X axis\nanchor.stick=112,140,left,Stick\n"
            + "anchor.button3=384,174,right,Button 3\nanchor.pov90=224,215,left,Hat right\n");
    write(
        sb.resolve(BOARD_DIR + "/StickControl.java"),
        "public class StickControl implements ControlBoard {\n"
            + "  CommandJoystick stick = new CommandJoystick(0);\n\n"
            + method("double", "translationX", "stick.getX()")
            + method("Trigger", "intakeOn", "stick.button(3)")
            + method("Trigger", "shoot", "stick.pov(90)")
            + "}\n");
    Path configs = sb.resolve(CONTROLS + "/configs.properties");
    write(
        configs,
        Files.readString(configs).replace("configs=test", "configs=test,stick")
            + "stick.title=Stick\nstick.source="
            + BOARD_DIR
            + "/StickControl.java\nstick.controllers=stick\nstick.stick.port=0\n"
            + "stick.stick.role=Pilot\nstick.stick.image=stick/v1\n");
    expectOk(gen(sb, "generate"));
    String md = read(sb, "diagrams/stick/v1/bindings.md");
    expect(md.contains("| Stick (X axis) | Drive left / right"), "getX() not mapped:\n" + md);
    expect(md.contains("| Button 3 | Intake on"), "button(3) not mapped:\n" + md);
    expect(md.contains("| Hat right | Shoot"), "pov(90) not mapped:\n" + md);
  }

  static void addSource() throws Exception {
    Path sb = sandbox();
    Path pic = sb.resolve("photo.svg");
    Files.copy(sb.resolve(CONTROLS + "/controller-sources/xbox/xbox-drawing.svg"), pic);
    expectOk(gen(sb, "add-source", "newpad", pic.toString()));
    Path saved = sb.resolve(CONTROLS + "/controller-sources/newpad/photo.svg");
    expect(Files.mismatch(pic, saved) == -1, "source must be an exact copy");
    Result again = gen(sb, "add-source", "newpad", pic.toString());
    expect(again.exit() != 0, "second add-source of the same file must be refused");
  }

  static void addLayout() throws Exception {
    Path sb = sandbox();
    Path pic = sb.resolve(CONTROLS + "/controller-sources/xbox/xbox-drawing.svg");
    expectOk(gen(sb, "add-layout", "xbox", pic.toString(), "--from", "xbox/v1"));
    Path v2 = sb.resolve(CONTROLS + "/controller-layouts/xbox/v2");
    Properties p = load(v2.resolve("anchors.properties"));
    expect(Files.exists(v2.resolve("controller.svg")), "v2 picture missing");
    expect("500".equals(p.getProperty("width")), "width not read from image");
    expect("340".equals(p.getProperty("height")), "height not read from image");
    expect(p.getProperty("source") == null, "source= must be reset for the new layout");
    expect(p.getProperty("anchor.a") != null, "anchors not copied from xbox/v1");
  }

  static void sandboxSvg() throws Exception {
    Path sb = sandbox();
    expectOk(gen(sb, "generate"));
    parseXml(sb.resolve(CONTROLS + "/diagrams/test/v1/diagram.svg"));
    expectOk(gen(sb, "preview-layout", "xbox/v1"));
    parseXml(sb.resolve("build/control-layouts/xbox-v1.svg"));
  }

  static void signOff() throws Exception {
    Path sb = sandbox();
    expectOk(gen(sb, "generate"));
    expect(read(sb, MASTER).contains("Human check: **not yet checked**"), "v1 not marked");
    expect(gen(sb, "sign-off", "test").exit() != 0, "sign-off without --by must fail");
    expectOk(gen(sb, "sign-off", "test", "--by", "Test Person", "--note", "drive team ok"));
    expect(
        read(sb, MASTER).contains("checked by Test Person ("),
        "MASTER missing the sign-off:\n" + read(sb, MASTER));
    expectOk(gen(sb, "check"));

    Path impl = sb.resolve(BOARD_DIR + "/TestControl.java");
    write(impl, Files.readString(impl).replace("return driver.b();", "return driver.y();"));
    expectOk(gen(sb, "generate"));
    String master = read(sb, MASTER);
    expect(master.contains("Human check: **not yet checked**"), "v2 should need a new check");
    expect(master.contains("checked by Test Person"), "v1 history lost its sign-off");
  }

  static void reviewPage() throws Exception {
    Path sb = sandbox();
    expectOk(gen(sb, "generate"));
    expectOk(gen(sb, "review"));
    String html = Files.readString(sb.resolve("build/control-review/index.html"));
    expect(html.contains("Test Controller - v1"), "review page missing the diagram");
    expect(html.contains("../../docs/controls/diagrams/test/v1/diagram.svg"), "bad diagram path");
    expect(html.contains("<h3>xbox/v1</h3>"), "review page missing the layout");
    expect(html.contains("sign-off test --by"), "review page missing sign-off command");
    expect(html.contains("Driver Station USB tab"), "review page missing on-robot checks");
    expect(html.contains("<code>climb()</code>"), "review page should list climb as unused");
  }

  /**
   * A tiny fake repo: one config on the real xbox/v1 layout. Bindings: A = intakeOn + outtake
   * (conflict), B = shoot, X = climb (never used by RobotContainer), turbo has no button.
   */
  static Path sandbox() throws IOException {
    Path sb = Files.createTempDirectory("control-diagram-test");
    sandboxes.add(sb);
    Path controls = sb.resolve(CONTROLS);
    Files.createDirectories(controls.resolve("tools"));
    copyTree(
        repo.resolve(CONTROLS + "/controller-layouts/xbox/v1"),
        controls.resolve("controller-layouts/xbox/v1"));
    copyTree(
        repo.resolve(CONTROLS + "/controller-sources/xbox"),
        controls.resolve("controller-sources/xbox"));
    write(
        controls.resolve("configs.properties"),
        "configs=test\ntest.title=Test Controller\ntest.source="
            + BOARD_DIR
            + "/TestControl.java\ntest.controllers=driver\ntest.driver.port=0\n"
            + "test.driver.role=Driver\ntest.driver.image=xbox/v1\n");
    write(
        controls.resolve("functions.properties"),
        "groups=Drive,Intake\ngroup.Drive=#1c64d1\ngroup.Intake=#15803d\n"
            + "fn.translationX=Drive|Drive left / right\nfn.intakeOn=Intake|Intake on\n"
            + "fn.outtake=Intake|Outtake\nfn.shoot=Intake|Shoot\nfn.climb=Intake|Climb\n"
            + "fn.turbo=Drive|Turbo\n");
    write(
        sb.resolve(BOARD_DIR + "/ControlBoard.java"),
        "public interface ControlBoard {\n  public double translationX();\n\n"
            + "  public Trigger intakeOn();\n\n  public Trigger outtake();\n\n"
            + "  public Trigger shoot();\n\n  public Trigger climb();\n\n"
            + "  public Trigger turbo();\n}\n");
    write(
        sb.resolve(BOARD_DIR + "/TestControl.java"),
        "public class TestControl implements ControlBoard {\n"
            + "  CommandXboxController driver = new CommandXboxController(0);\n\n"
            + method("double", "translationX", "driver.getLeftX()")
            + method("Trigger", "intakeOn", "driver.a()")
            + method("Trigger", "outtake", "driver.a()")
            + method("Trigger", "shoot", "driver.b()")
            + method("Trigger", "climb", "driver.x()")
            + method("Trigger", "turbo", "null")
            + "}\n");
    write(
        sb.resolve("src/main/java/org/jmhsrobotics/frc2026/RobotContainer.java"),
        "class RobotContainer {\n  void bind() {\n    control.translationX();\n"
            + "    control.intakeOn();\n    control.outtake();\n    control.shoot();\n"
            + "    control.turbo();\n    // control.climb(); commented out = not wired\n  }\n}\n");
    return sb;
  }

  /** The bindings.md table row for an input + function label, or fail. */
  static String row(String md, String input, String label) {
    return md.lines()
        .filter(l -> l.startsWith("| " + input + " | " + label + " `"))
        .findFirst()
        .orElseThrow(() -> new AssertionError("no row for " + input + " / " + label + ":\n" + md));
  }

  static String method(String type, String name, String body) {
    return "  @Override\n  public " + type + " " + name + "() {\n    return " + body + ";\n  }\n\n";
  }

  // ===================== helpers =====================

  static Result gen(Path cwd, String... args) throws IOException, InterruptedException {
    List<String> cmd = new ArrayList<>();
    cmd.add(Path.of(System.getProperty("java.home"), "bin", "java").toString());
    cmd.add(generator.toString());
    cmd.addAll(List.of(args));
    Process p = new ProcessBuilder(cmd).directory(cwd.toFile()).redirectErrorStream(true).start();
    String out = new String(p.getInputStream().readAllBytes(), StandardCharsets.UTF_8).trim();
    return new Result(p.waitFor(), out);
  }

  static void expectOk(Result r) {
    expect(r.exit() == 0, "generator failed:\n" + r.output());
  }

  static void expect(boolean ok, String message) {
    if (!ok) throw new AssertionError(message);
  }

  static void parseXml(Path file) throws Exception {
    try {
      DocumentBuilderFactory.newInstance().newDocumentBuilder().parse(file.toFile());
    } catch (Exception e) {
      throw new AssertionError(repo.relativize(file.toAbsolutePath()) + ": " + e.getMessage());
    }
  }

  static String read(Path sandbox, String rel) throws IOException {
    return Files.readString(sandbox.resolve(CONTROLS).resolve(rel));
  }

  static void write(Path file, String content) throws IOException {
    Files.createDirectories(file.getParent());
    Files.writeString(file, content);
  }

  static void copyTree(Path from, Path to) throws IOException {
    try (Stream<Path> s = Files.walk(from)) {
      for (Path p : s.sorted(Comparator.naturalOrder()).collect(Collectors.toList())) {
        Path dest = to.resolve(from.relativize(p).toString());
        if (Files.isDirectory(p)) Files.createDirectories(dest);
        else Files.copy(p, dest);
      }
    }
  }

  static void deleteTree(Path dir) throws IOException {
    try (Stream<Path> s = Files.walk(dir)) {
      for (Path p : s.sorted(Comparator.reverseOrder()).collect(Collectors.toList())) {
        Files.deleteIfExists(p);
      }
    }
  }

  static Properties load(Path file) throws IOException {
    Properties p = new Properties();
    try (var r = Files.newBufferedReader(file)) {
      p.load(r);
    }
    return p;
  }
}
