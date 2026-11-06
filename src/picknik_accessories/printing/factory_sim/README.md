# factory_sim 3d printing assets

The `factory_sim` example workspace models a small factory environment. The ["Pick and Place Brackets from Left Bin"](https://github.com/PickNikRobotics/moveit_pro_example_ws/blob/main/src/factory_sim/objectives/pick_and_place_brackets_from_left_bin.xml) Objective models an automotive-like bracket part for the robot to pick, place on a jig, and the deposit into a bin.

![Pick and Place Brackets from Left Bin](images/pick_and_place_brackets_from_left_bin_objective_preview1.png)

In order to test this on robot hardware, several parts need to be 3d printed. These files are saved as 3mf files as that is the new standard mesh file in 3d printing and does a better job preserving properties of the model than STLs. STEP files of the jig are included for anyone who wants to modify the parts.

## Bracket Part
![Bracket Part](images/bracket_part1.jpg)
This simulated automotive bracket can be found in [`3mf/pick_and_place_bracket_part.3mf`](3mf/pick_and_place_bracket_part.3mf)
It was printed on a Bambu Labs X1 Carbon out of PLA with a 0.4 mm nozzle with 15% infill and 2 perimeters, which are the default settings for that machine. As this part isn't expected to incur heavy loads, no special settings should be required.

The part was designed to be picked with a vacuum gripper using a 20 mm diameter nozzle, but other nozzle diameters may work as there are many flat areas that could be picked.

## Bracket Jig
The simulated bracket jig is modelled as two tapered pegs. Unfortunately, the real world does not allow us to spawn tapered pegs anywhere in the world, so we are required to model more geometry than that 😅.

![The jig with both pegs seated](images/jig_with_pegs.png)

The jig has two 11 mm diameter tapered pegs, spaced 65.0 mm apart on center, which matches the posts in the `factory_sim` scene. It is made of two parts, a base and a peg (print two pegs), so the base prints flat and the pegs print lying down. Each peg is 86.5 mm long and stands 75 mm above its boss once seated.

| Part | 3MF (print) | STEP (edit) | Preview |
| --- | --- | --- | --- |
| Base, 207 x 157 x 12 mm | [3MF](3mf/pick_and_place_bracket_jig_base.3mf) | [STEP](step/pick_and_place_bracket_jig_base.step) | <img src="images/preview_base.png" width="160" alt="Preview of the jig base"> |
| Peg, print two | [3MF](3mf/pick_and_place_bracket_jig_peg.3mf) | [STEP](step/pick_and_place_bracket_jig_peg.step) | <img src="images/preview_peg.png" width="160" alt="Preview of the jig peg"> |
| Whole plate: base and two pegs, with the print settings below | [3MF](3mf/pick_and_place_bracket_jig_plate.3mf) | | <img src="images/preview_plate.png" width="160" alt="Preview of the whole plate as it prints, the base with two pegs lying beside it"> |
| Base with both pegs seated | | [STEP](step/pick_and_place_bracket_jig_assembled.step) | <img src="images/preview_assembled.png" width="160" alt="Preview of the base with both pegs seated"> |

The 3MF part files are already turned the way they print. The STEP files are the as-designed solids, for changing the parts. The peg STEP is modelled standing up, with the shoulder at 10 mm and the tip at 85 mm, and its axis is at the origin. To position a standalone peg in the base, translate it by (76.3624, -79.8261, 0) mm or (141.3005, -82.6614, 0) mm, which are the two socket axes in the base STEP's coordinates. The assembled STEP already contains both positioned pegs.

### Print settings
The base and both pegs fit on one plate, [`3mf/pick_and_place_bracket_jig_plate.3mf`](3mf/pick_and_place_bracket_jig_plate.3mf), and need no supports. It was sliced for a Bambu Labs X1 Carbon with a 0.4 mm nozzle on a textured PEI plate:

- PLA, one filament
- 0.16 mm layers on a 0.2 mm first layer
- 2 walls and 100% infill
- **Supports off**, no brim
- Nozzle 220°C, bed 55°C
- The base prints flat, upright. The pegs lie on their flat side beside it.
- About 2 h 40 min and 117 g for the plate: roughly 104 g for the base and 7 g for each peg

Studio warns that a 55°C bed is warm for PLA in a closed chamber, so leave the printer door open. To change the plate type, the filament or a setting, open the 3MF, change it and slice again. Leave supports off, and leave the seam setting on the base at *Back* so the seam lands in the socket's notch.

Print the pegs lying on their flat, as the plate does: that is how they print with no supports, and the slit stays square to the bed. A plate this size can lift at its corners; if it does, add a brim.

> [!NOTE]
> The bracket jig base contains a number of holes and cutouts from the sides. As this is a large flat print warping could occur and these cutouts deter this from happening.

## Assembly
Each peg is a slit peg that clicks into a socket in its boss.

![Peg lifted out of its socket](images/jig_peg_lifted_from_socket.png)

**The peg.** The peg has a flat down one side (the side it prints on), a shoulder, and a round plug below the shoulder. A 0.4 mm slit runs from the plug end up to the shoulder and splits the plug into two prongs that can flex toward each other. A keel on the plug is a rib that fits the socket's keyway. A bead on each prong, 7.0 mm across, stands 0.1 mm proud of the 6.8 mm socket bore.

**The socket.** Each boss is 11 mm round and has a 6.8 mm round socket through it and the floor of the base. The socket has a keyway for the keel, and a groove 7.2 mm across just above the floor. A small notch on the back wall is where the slicer starts the wall, so the seam stays out of the joint.

**How it clicks.** As the peg goes in, the sloped front of each bead meets the socket wall and squeezes the two prongs together, closing the slit. When the shoulder reaches the boss, the beads are level with the groove and spring out into it, which you feel and hear as a click. Pulling the peg out drags the sloped back of each bead over the groove's edge, so it takes a firm pull by hand, which keeps the peg seated when the jig is handled.

**Bead and groove, the joint.** Cut through the peg in its socket, across the slit. The cut is level with the print bed, so you look down on one layer of the peg.

![Cross-section of the peg seated in its socket, with the catch enlarged at right](images/jig_joint_section.png)

Drawn to scale. The enlarged view of the catch is 5.5 times bigger.

**On the print bed.** Cut across the peg at the catch, seen from the plug end. Layer lines are drawn wider apart than real ones.

![Cross-section of the peg as it lies on the print bed, showing the bead and the 45 degree underside](images/jig_joint_print_bed.png)

1. Peg body, 11 mm.
2. Slit, 0.4 mm wide, from the plug end to the shoulder.
3. A bead on each prong, 7.0 mm across.
4. Groove round the inside of the socket, 7.2 mm across, in the foot of the boss. 1.9 mm of wall is left behind it.
5. Seated, the bead sits in the groove with nothing squeezing it. Its sloped back has to climb out past the groove's edge.
6. The bead stands 0.1 mm past the socket wall. That is the catch.
7. On the bed the bead sits at half height. Its underside is the keel's 45 degree side carried further out, so nothing overhangs.
8. The strip each prong prints on, 0.9 mm wide.

| Joint dimension | Value |
| --- | --- |
| Peg diameter | 11 mm |
| Peg length | 86.5 mm |
| Slit | 0.4 mm |
| Bead, across the pair | 7.0 mm |
| Socket bore | 6.8 mm |
| Groove | 7.2 mm |
| Peg spacing | 65.0 mm, center to center |

**One way only.** The keyway faces the front of the printer as the base lies on the bed, and the notch faces the back. The keel only goes into the keyway, so a peg fits in one orientation, and both pegs stand with their flat toward the front edge of the base.

To assemble:
1. Look along each slit toward the light. It should be open from the plug end to the shoulder. If the first layer has tacked it shut, run a knife blade along it.
2. Line up the keel with the keyway and push the peg straight in until the shoulder sits on the boss all round. It clicks when it seats. It should go in by hand with a firm push, so if it resists, check the orientation.
3. To remove a peg, pull it straight out by hand. If one will not come, push its plug end up from underneath with a pen, since the sockets are open through the floor. If the base is screwed down, lift it off first.

## Mounting
The jig base has five countersunk holes that can be used to mount onto a flat surface, using a common, general purpose #2 phillips drive flat head screw. The holes are 4.5 mm clear with a 10 mm countersink on the top face. Five screws are not needed to secure the base, but rather 3 of the screws are spaced 50 mm and 100 mm apart to facilitate precise locating on a table. The diagram below shows the hole spacing on the base.

![Top view of the jig base with the mounting hole spacing, 50 mm and 100 mm](images/jig_mounting_diagram.png)

The five large round holes are for weight and warping only, not for mounting.

## Use
Once assembled, the bracket part should slide easily over the pegs and come to rest in the seat of the jig.  

![A bracket seated on the jig](images/bracket_on_jig.png)

*A render of the bracket part seated on the jig, with both pegs through its holes.*

## Bins
We recommend using the [ULINE Straight Wall Container 24" x 15" x 7 1⁄2"](https://www.uline.com/Product/ProductDetailRootItem?modelnumber=S-19509) for recreating the pick and place demo, which is what the bins in the simulation are modelled after.  

![ULINE Bin](images/uline_bin1.png)
