"""Experiment: a test game for the arm that unstacks and restacks cubes. No real purpose except testing things.

The cube stacks are stored in a 9 x 9 matrix (the db): db[column][row].
  column 0..8, counted from the left (the three towers start at columns 3, 4, 5; 0-2 and 6-8 are free spots).
  row    0..8, counted upwards from the table (row 0 is the cube on the table).
A cell holds the cube there (None = empty).
"""

import json
import os
import select
import subprocess
import sys
import termios
import tty

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory

from gz.msgs10.empty_pb2 import Empty
from gz.transport13 import Node as GzNode

from gr_blocks_ros.arm_mover import ArmMover

COLUMNS = 9
ROWS = 9

# Where the cubes are, in base_link (m). From worlds/blocks_world_long.sdf: the towers stand at x = 1.0, columns 0.12 m apart
# (column 3 at y = +0.12, 4 at y = 0, 5 at y = -0.12, so column c is at y = 0.12 * (4 - c)). The table top is z = 0 in
# base_link; a cube is 0.045 m high and the cubes of a tower are 0.0455 m apart (0.0005 m gap), the first centre at z = 0.023.
TOWER_X = 1.0
COLUMN_SPACING = 0.12
FIRST_CUBE_Z = 0.023
ROW_SPACING = 0.0455
CUBE_SIZE = 0.045
TCP_OFFSET = 0.177  # m, from end_effector_link along its approach axis to the finger pads (URDF)

# The three towers at the start, bottom to top, by column (worlds/blocks_world_long.sdf); each letter is the model block_<letter>.
TOWERS = ((3, "RETH"), (4, "OMSL"), (5, "ABWF"))
WORLD = "blocks_world"  # Gazebo world name
BASE_Z = 0.30  # m, height of base_link in the Gazebo world (the robot is spawned on the table top)

OPEN_MM = 85.0  # gripper opening (contact-surface gap) while going to a target
TARGET_DURATION = 4.0  # s, the arm's move from observe to a target
GRIPPER_OPEN_FRACTION = 0.4  # the gripper opens within this fraction of TARGET_DURATION (it must finish before 50 %)
CLOSE_MM = 52.0  # gripper opening when closed on a cube
CLOSE_DURATION = 1.0  # s, closing the gripper on the cube
LIFT_M = 0.10  # m, lift after closing on the cube
LIFT_DURATION = 2.0  # s
PLACE_CLEARANCE = 0.02  # m, the held cube arrives this far above its place and is then lowered onto the stack
PLACE_LOWER_DURATION = 1.0  # s
RETREAT_M = 0.10  # m, after releasing: straight back along -x, then to observe
RETREAT_DURATION = 1.5  # s
BACK_M = 0.30  # m, back away from the table (-x) after the lift
BACK_DURATION = 3.0  # s


def load_vocabulary():
    """The words of the paper (the goals in data/blockworld.json), e.g. HER, MOTHER, MASTER."""
    with open(os.path.join(get_package_share_directory("gr_blocks_ros"), "data", "blockworld.json")) as f:
        return list(json.load(f)["goals"])


def read_line(prompt):
    """Read a line from the terminal; returns None when Esc is pressed (arrow keys and other escape sequences are ignored)."""
    sys.stdout.write(prompt)
    sys.stdout.flush()
    if not sys.stdin.isatty():  # piped input: no keys, just lines (None at the end of the input)
        line = sys.stdin.readline()
        return line.rstrip("\n") if line else None
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)  # keys arrive one by one; Ctrl+C still works
        text = ""
        while True:
            ch = os.read(fd, 1).decode(errors="ignore")
            if ch == "\x1b":
                if select.select([fd], [], [], 0.05)[0]:  # more bytes follow: an arrow key etc., not Esc; drop it
                    while select.select([fd], [], [], 0.01)[0]:
                        os.read(fd, 1)
                    continue
                print()
                return None
            if ch in ("\r", "\n"):
                print()
                return text
            if ch in ("\x7f", "\b"):
                if text:
                    text = text[:-1]
                    sys.stdout.write("\b \b")
            elif ch.isprintable() and ch:
                text += ch
                sys.stdout.write(ch)
            sys.stdout.flush()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


class Experiment:
    def __init__(self):
        self._db = [[None] * ROWS for _ in range(COLUMNS)]  # db[column][row]
        # The three towers of worlds/blocks_world_long.sdf, bottom to top. Columns count from the left as the robot sees
        # the table (the robot faces +x, so its left is +y): column 3 is the tower at y = +0.12, 4 at y = 0, 5 at y = -0.12.
        for column, tower in TOWERS:
            for row, letter in enumerate(tower):
                self._db[column][row] = letter
        # positions[column][row] = (x, y, z) of the centre of a cube at that cell, in base_link (not joint angles)
        self._positions = [[(TOWER_X, COLUMN_SPACING * (4 - column), FIRST_CUBE_Z + ROW_SPACING * row)
                            for row in range(ROWS)] for column in range(COLUMNS)]
        self._gz = GzNode()  # sim only: the attach / detach topics of the cubes' DetachableJoint plugins (see attach)
        letters = [letter for _, tower in TOWERS for letter in tower]
        self._attach_pub = {letter: self._gz.advertise(f"/gr_blocks/attach/{letter}", Empty) for letter in letters}
        self._detach_pub = {letter: self._gz.advertise(f"/gr_blocks/detach/{letter}", Empty) for letter in letters}
        self._attached = None  # the letter of the cube attached to the gripper, if any
        self._vocabulary = load_vocabulary()
        self._word = None  # chosen in start()
        self._arm = ArmMover()

    def attach(self, cube):
        """Attach cube `cube` (its letter) to the gripper: from now on it keeps the position and orientation relative to the
        gripper that it has at this moment, until release(cube). Only one cube can be attached at a time.

        Sim only: a message to the cube's Gazebo DetachableJoint ("weld": true in gr_blocks.json); nothing happens without it.
        """
        if self._attached is not None:
            raise RuntimeError(f"cube {self._attached} is already attached to the gripper: release it first")
        self._attach_pub[cube].publish(Empty())
        self._attached = cube

    def release(self, cube):
        """Undo attach(cube): the cube is left where it is and falls or rests under physics again."""
        if self._attached != cube:
            raise ValueError(f"cube {cube} is not attached to the gripper")
        self._detach_pub[cube].publish(Empty())
        self._attached = None

    def _go_observe(self):
        """Every move starts and ends here: the arm at the `observe` pose (config/poses.json), the gripper open to OPEN_MM
        (it is never fully closed)."""
        with open(os.path.join(get_package_share_directory("gr_blocks_ros"), "config", "poses.json")) as f:
            observe = json.load(f)["observe"]
        arm = self._arm.move_fk(observe, 3.0)
        gripper = self._arm.move_gripper_dist(OPEN_MM, 1.0)
        arm.wait()
        gripper.wait()

    def _reset_cubes(self):
        """Teleport every cube to its starting place (Gazebo set_pose), whether or not it was moved.

        Needs the sim in the same GZ_PARTITION as this process (the usual exports).
        """
        for column, tower in TOWERS:
            for row, letter in enumerate(tower):
                x, y, z = self._positions[column][row]
                request = (f'name: "block_{letter}" position: {{x: {x} y: {y} z: {z + BASE_Z}}} '
                           'orientation: {x: 0 y: 0 z: 0 w: 1}')
                result = subprocess.run(
                    ["gz", "service", "-s", f"/world/{WORLD}/set_pose", "--reqtype", "gz.msgs.Pose",
                     "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", request],
                    capture_output=True, text=True, timeout=10)
                if "true" not in result.stdout:
                    raise RuntimeError(f"could not teleport block_{letter}: {result.stdout.strip() or result.stderr.strip()}")

    def start(self):
        """Move to `observe` with the gripper open, put the cubes in their starting places, then ask the user to pick a
        word from the paper vocabulary (the goal). Returns the word, or None if the user pressed Esc."""
        self._go_observe()
        self._reset_cubes()
        print("Pick a word:", ", ".join(self._vocabulary))
        while True:
            line = read_line("word (Esc to exit)> ")
            if line is None:
                return None
            word = line.strip().upper()
            if word in self._vocabulary:
                self._word = word
                return word
            print(f"{word!r} is not in the vocabulary")

    def show_board(self):
        """Print the db: row 0 (the table) at the bottom, columns 0..8 from the left."""
        for row in range(ROWS - 1, -1, -1):
            print(f"{row} | " + " ".join(self._db[column][row] or "." for column in range(COLUMNS)))
        print("    " + "-" * (2 * COLUMNS - 1))
        print("    " + " ".join(str(column) for column in range(COLUMNS)))

    def ask_move(self):
        """Ask the user for a move as "SRC DEST" columns (e.g. "3 0"); returns (src_column, dest_column), or None on Esc."""
        while True:
            line = read_line("move (SRC_COLUMN DEST_COLUMN, Esc to exit)> ")
            if line is None:
                return None
            parts = line.split()
            try:
                src, dest = int(parts[0]), int(parts[1])
                if len(parts) == 2 and 0 <= src < COLUMNS and 0 <= dest < COLUMNS:
                    return src, dest
            except (IndexError, ValueError):
                pass
            print(f"expected: two columns 0-{COLUMNS - 1}, e.g. 3 0")

    def is_done(self):
        """True when the word is created in one of the columns.

        As in the paper (goal = "X on Y" pairs, last letter on the table), a column spells the word when, from the table
        upwards, it holds the word's letters last to first: the last letter on the table, the first letter on top of the
        stack. (HER: R on the table, E on R, H on E.)
        """
        letters = list(reversed(self._word))
        return any(self._db[column][: len(letters)] == letters for column in range(COLUMNS))

    def _height(self, column):
        """Number of cubes in a column."""
        return sum(cube is not None for cube in self._db[column])

    def _reach(self, cube):
        """From observe: open the gripper and point at `cube` (x, y, z, the centre of a cube cell), reaching it with the pad frame on its front face.
        Returns the end effector position it reached.

        The gripper opens to OPEN_MM within GRIPPER_OPEN_FRACTION of the arm's move, which starts at the same time. The
        gripper is pointed horizontally along +x at the cell, square to the cube faces (the cubes are not rotated). The
        first 50 % of the way (and of the time) lines the gripper up with the target; the last 50 % is a straight line to it.
        """
        cube = np.asarray(cube, float)
        approach = np.array([1.0, 0.0, 0.0])  # along +x: square to the front face of every cube, so the pads are parallel to its sides
        position = cube - approach * (CUBE_SIZE / 2 + TCP_OFFSET)
        gripper = self._arm.move_gripper_dist(OPEN_MM, GRIPPER_OPEN_FRACTION * TARGET_DURATION)
        arm = self._arm.move_ik(position, target=cube, pointing=True, align=True, duration=TARGET_DURATION)
        arm.wait()
        gripper.wait()
        return position

    def play(self, src_column, dest_column):
        """Test move: the arm reaches the top cube of `src_column` with the gripper open, closes the gripper (the cube is attached),
        lifts 10 cm, backs up 30 cm, reaches the place for it on top of the stack in `dest_column` (on the table if empty) with the
        cube held, releases the cube and opens the gripper, updates the db, backs up 10 cm along -x and goes back to observe.

        The gripper opens on the way to the cube (see _reach) and stays open (OPEN_MM): the move ends at observe with the arm
        and gripper as every move starts. If the move fails, a cube that is attached is released and the arm goes back to observe before the error is raised.
        """
        if not self._db[src_column][0]:
            raise ValueError(f"column {src_column} is empty: nothing to pick")
        place_row = self._height(dest_column) - (1 if dest_column == src_column else 0)  # the picked cube is gone from its column
        if place_row >= ROWS:
            raise ValueError(f"column {dest_column} is full")
        try:
            pick_row = self._height(src_column) - 1
            letter = self._db[src_column][pick_row]
            cube = np.array(self._positions[src_column][self._height(src_column) - 1])
            position = self._reach(cube)
            self._arm.move_gripper_dist(CLOSE_MM, CLOSE_DURATION).wait()  # close on the cube
            self.attach(letter)  # the cube now keeps its place relative to the gripper
            up = np.array([0.0, 0.0, LIFT_M])  # lift: same aim (+x), everything LIFT_M higher
            self._arm.move_ik(position + up, target=cube + up, pointing=True, straight=True, duration=LIFT_DURATION).wait()
            back = np.array([-BACK_M, 0.0, 0.0])  # back up: same aim (+x), everything BACK_M further from the table
            self._arm.move_ik(position + up + back, target=cube + up + back, pointing=True, straight=True, duration=BACK_DURATION).wait()
            # the destination: the held cube goes where the cell centre `dest` is, so the end effector is where it was relative to the
            # cube at the pick (CUBE_SIZE / 2 + TCP_OFFSET behind it along +x); the same line-up and straight approach as the reach,
            # but PLACE_CLEARANCE higher, followed by a straight move down onto the stack
            dest = np.array(self._positions[dest_column][place_row])
            ee = dest - np.array([CUBE_SIZE / 2 + TCP_OFFSET, 0.0, 0.0])
            above = np.array([0.0, 0.0, PLACE_CLEARANCE])  # arrive above the place, so the held cube does not scrape the stack top
            self._arm.move_ik(ee + above, target=dest + above, pointing=True, align=True, duration=TARGET_DURATION).wait()
            self._arm.move_ik(ee, target=dest, pointing=True, straight=True, duration=PLACE_LOWER_DURATION).wait()  # lower it onto the stack
            self.release(letter)  # undo the attach just before opening the gripper: the cube stays on the stack
            self._arm.move_gripper_dist(OPEN_MM, GRIPPER_OPEN_FRACTION * TARGET_DURATION).wait()
            self._db[src_column][pick_row] = None  # the cube is now on top of the destination stack
            self._db[dest_column][place_row] = letter
            retreat = np.array([-RETREAT_M, 0.0, 0.0])  # back away from the stack parallel to the x axis before leaving for observe
            self._arm.move_ik(ee + retreat, target=dest + retreat, pointing=True, straight=True, duration=RETREAT_DURATION).wait()
            self._go_observe()
        except RuntimeError:
            if self._attached:  # a failed move must not leave a cube attached: it would block every later move
                self.release(self._attached)
            try:
                self._go_observe()
            except RuntimeError as error:
                print(f"could not return to observe: {error}")
            raise


def main():
    """python3 -m gr_blocks_ros.experiment   (the sim must be running, see docs/HANDOFF_SESSION4.md)"""
    rclpy.init()
    experiment = Experiment()
    if experiment.start() is None:  # Esc
        return
    while True:
        experiment.show_board()
        move = experiment.ask_move()
        if move is None:  # Esc
            break
        try:
            experiment.play(*move)
        except (RuntimeError, ValueError) as error:  # e.g. no IK solution (out of reach), empty source column
            print(f"move failed: {error}")
        if experiment.is_done():  # the word is created in one of the columns
            experiment.show_board()
            print(f"{experiment._word} is built. Game over.")
            break


if __name__ == "__main__":
    main()
