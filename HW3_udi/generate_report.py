from reportlab.lib.pagesizes import LETTER
from reportlab.pdfgen import canvas


def add_lines(c, lines, x=54, y=740, leading=16):
    text = c.beginText(x, y)
    text.setFont("Helvetica", 11)
    for line in lines:
        if line == "<PAGE_BREAK>":
            c.drawText(text)
            c.showPage()
            text = c.beginText(x, 740)
            text.setFont("Helvetica", 11)
            continue
        text.textLine(line)
    c.drawText(text)


def main() -> None:
    c = canvas.Canvas("HW3_report.pdf", pagesize=LETTER)

    lines = [
        "PHYSICS 2VG3 - Homework 3 Report",
        "",
        "Exercise 1 - Off-centre collision between two equal masses",
        "Setup: m1=m2=1.0, r1=r2=1.0",
        "Initial positions: (-5,25,0.4) and (5,25,-0.4)",
        "Initial velocities: v1=(2,0,0), v2=(-1,0,0)",
        "",
        "Qualitative outcome:",
        "The two equal masses exchange their normal velocity components at contact.",
        "The left mass deflects upward in +z and moves left; the right mass deflects downward in -z and moves right.",
        "",
        "Numerical results from collision_2mass.py:",
        "Before collision:",
        "  P = (1, 0, 0)",
        "  L = (0, 1.2, -25)",
        "  K = 2.5",
        "Immediately after first collision:",
        "  P = (1, 0, 0)",
        "  L = (0, 1.2002916688, -25)",
        "  K = 2.4999999999999987",
        "After simulation:",
        "  P = (1, 0, 0)",
        "  L = (0, 1.2002916688, -25)",
        "  K = 2.4999999999999987",
        "Final velocities:",
        "  v_left  = (-0.5197666366, 0, 1.10003454804)",
        "  v_right = ( 1.5197666366, 0, -1.10003454804)",
        "",
        "Exercise 2 - Basketball, ping-pong ball, wall",
        "Setup: inverse masses = 1.0, 0.01, 0.0 (wall)",
        "Why inverse mass of wall is 0: infinite mass means zero acceleration under finite impulse,",
        "so the wall remains fixed (physically appropriate rigid boundary).",
        "",
        "Given positions/radii/velocities from assignment:",
        "Final velocities from collision_3masses.py baseline:",
        "  v1 (ping-pong)  = (-29.603960396, 0, 0)",
        "  v2 (basketball) = (-9.60396039604, 0, 0)",
        "  v3 (wall)       = (0, 0, 0)",
        "As multiples of initial vx=10:",
        "  v1/vx = -2.9603960396039604",
        "  v2/vx = -0.9603960396039604",
        "  v3/vx = 0",
        "",
        "Sequence of events:",
        "1) The basketball (mass 100) hits the wall and reverses direction.",
        "2) It then collides head-on with the ping-pong ball (mass 1).",
        "3) Elastic collision transfers large speed change to the light ball.",
        "",
        "Minor setup changes (no overlap, same order, same initial speed) affect timing only;",
        "the normalized final speeds v1/vx and v2/vx remain the same in these tests.",
        "<PAGE_BREAK>",
        "General symbolic result for any initial vx=u>0 with masses m (left) and M (middle):",
        "  v1 = ((m - 3M)/(m + M)) u",
        "  v2 = ((3m - M)/(m + M)) u",
        "  v3 = 0",
        "Limiting case m << M (very light first mass):",
        "  v1 -> -3u, v2 -> -u, v3 = 0",
        "",
        "Exercise 3 - Many random particles in a 3D box",
        "Code file: collsion_many.py (12 particles, random seed=2)",
        "All simulations now run as visual Panda3D apps (ShowBase + task loop).",
        "Walls used at x,z in [-20,20], y in [80,120], with elastic bounce in all 3 directions.",
        "",
        "Conserved quantities printed by code:",
        "Start (t=0):",
        "  P = (7.21969973911, 39.6792874611, 1.8345113423)",
        "  L = (287.697232866, 168.487732251, -195.501279954)",
        "  K = 669.8171214683324",
        "After 30 s:",
        "  P = (-23.9514357434, -25.062591668, -35.3678347096)",
        "  L = (-3989.53484698, 221.370669718, 2735.20412379)",
        "  K = 669.8171214683325",
        "",
        "What is conserved now:",
        "Kinetic energy is conserved (elastic particle-particle and particle-wall collisions).",
        "Linear momentum is not conserved for the particle subsystem because walls exert external impulses.",
        "Angular momentum is also not conserved for the subsystem because wall impulses provide external torque.",
        "",
        "Submitted code files:",
        "  panda_collision.py",
        "  collision_2masses.py",
        "  collision_2mass.py",
        "  collision_3masses.py",
        "  collsion_many.py",
    ]

    add_lines(c, lines)
    c.save()


if __name__ == "__main__":
    main()
