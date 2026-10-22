# Mechanical

Charara is a four-wheel **mecanum** platform: each drive module carries a
mecanum wheel through a printed gearbox on a stepper, and the frame mixes
3D-printed structural parts with aluminium plates.

Two sets of images:

- `CAD_photos/` - views exported from the CAD (the design).
- [`../Docs/assets/hardware/`](../Docs/assets/hardware/) - photographs of the
  robot as built (the result). The raw camera files stay out of git.

The geometry the software cares about - body size, clearance margin,
half-diagonal - is configured in the console, see
[`../Docs/04-console-guide.md`](../Docs/04-console-guide.md) section 8.

## CAD

<p align="center">
  <a href="CAD_photos/cad-isometric-left.png"><img width="49%" alt="Isometric view, electronics side" src="CAD_photos/cad-isometric-left.png"></a>
  <a href="CAD_photos/cad-isometric-right.png"><img width="49%" alt="Isometric view, drivetrain side" src="CAD_photos/cad-isometric-right.png"></a>
</p>
<p align="center"><sub><b>Isometric, electronics side</b> &nbsp;·&nbsp; <b>isometric, drivetrain side</b></sub></p>

<p align="center">
  <a href="CAD_photos/cad-front.png"><img width="32%" alt="Front elevation" src="CAD_photos/cad-front.png"></a>
  <a href="CAD_photos/cad-side.png"><img width="32%" alt="Side elevation" src="CAD_photos/cad-side.png"></a>
  <a href="CAD_photos/cad-top.png"><img width="32%" alt="Top view" src="CAD_photos/cad-top.png"></a>
</p>
<p align="center"><sub><b>Front</b> &nbsp;·&nbsp; <b>side</b> &nbsp;·&nbsp; <b>top</b></sub></p>

## Real photos

<p align="center">
  <a href="../Docs/assets/hardware/chassis-top.jpg"><img height="230" alt="Bare chassis from above: four mecanum wheels, steppers and the controller board" src="../Docs/assets/hardware/chassis-top.jpg"></a>
  <a href="../Docs/assets/hardware/drivetrain.jpg"><img height="230" alt="Drivetrain with the motors wired, seen from above" src="../Docs/assets/hardware/drivetrain.jpg"></a>
  <a href="../Docs/assets/hardware/robot.jpg"><img height="230" alt="The assembled robot on the floor" src="../Docs/assets/hardware/robot.jpg"></a>
</p>
<p align="center"><sub><b>Bare chassis</b> &nbsp;·&nbsp; <b>drivetrain, wired</b> &nbsp;·&nbsp; <b>assembled robot</b></sub></p>

<p align="center">
  <a href="../Docs/assets/hardware/bearing-mount.jpg"><img height="200" alt="Wheel hub: printed mount, bearing and coupling" src="../Docs/assets/hardware/bearing-mount.jpg"></a>
  <a href="../Docs/assets/hardware/stepper-coupling.jpg"><img height="200" alt="Spring-loaded odometry wheel: two tension springs hold the encoder wheel against the floor" src="../Docs/assets/hardware/stepper-coupling.jpg"></a>
  <a href="../Docs/assets/hardware/vision-setup.jpg"><img height="200" alt="Robot and camera on the field, with vision tags" src="../Docs/assets/hardware/vision-setup.jpg"></a>
</p>
<p align="center"><sub><b>Wheel hub</b> &nbsp;·&nbsp; <b>spring-loaded odometry wheel</b> &nbsp;·&nbsp; <b>robot and camera on the field</b></sub></p>

**Spring-loaded odometry wheels.** Two small encoder wheels ride on the table
next to the drive modules. Each one is carried by a sprung lever: the two
tension springs in the middle photo pull the wheel onto the floor, so it keeps
contact - and keeps measuring - through shocks, bumps and the small chassis
movements of a mecanum drive instead of skipping on a bad seam and losing the
odometry.