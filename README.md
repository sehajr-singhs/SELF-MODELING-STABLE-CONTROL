# Self-Modeling Stable Control

Sehaj Randhir Singh. Simulation only, MuJoCo, no hardware.

A controller is only ever as good as the model it was tuned against, and that
model goes stale the moment the machine starts to age, because friction climbs as
joints wear and windings heat, the payload changes between tasks, and the torque
you actually get for a given command sags as the motor warms, so the controller
that tracked tightly when it was commissioned is slowly steering a plant it no
longer matches. You can chase this with periodic recalibration, but that needs a
person and a stop, and it throws away the fact that the robot is already measuring
almost everything it would need to notice the change on its own. The question this
work asks, and answers in a controlled simulation, is whether a robot can estimate
its own slowly-changing dynamics online from nothing but its own proprioceptive
and electrical data, and feed that self-estimate into a controller whose learned
part is provably bounded, so tracking holds as the hardware drifts. The reason to
insist on the bound is that an unconstrained network bolted onto a controller can
track beautifully right up until it does something unsafe, and a robot that is
modeling itself in the loop has to stay inside guarantees it cannot talk itself
out of, which is what makes the learned correction something you would actually
trust on a machine.

The longer arc this points at is a robot that builds and maintains a usable
internal model of its own body from raw self-observation and keeps it current as
the body changes, the way a careful operator builds an intuition for a specific
machine and notices when it is off. That is the motivation rather than the claim
here, because what is demonstrated is the small, sharp first step, online
self-modeling of drift for control with a stability guarantee on the learned part,
on small systems, in simulation, and the honest value of the artifact is that
every number in it comes from a script that reads a saved file, so the wins and
the failures are both real.

## How it works, system before component

The controller has three layers sitting on top of a self-observation feed, and the
order matters because each layer only makes sense once the one under it is solid.
The bottom layer is a computed-torque baseline, which is inverse-dynamics control,
and it stays in charge of stability the entire time and is never replaced. At every
tick it asks a frozen nominal model of the robot for the mass matrix, the bias
force from gravity and Coriolis, and the passive force from nominal damping and
friction, then commands the torque that would produce the acceleration it wants,
which is the reference acceleration shaped by a PD term on the tracking error. When
the nominal model matches the plant this cancels the dynamics and leaves the linear
error system e double dot plus Kd e dot plus Kp e equal to zero, and that is why
the no-drift tracking can be driven down into the low thousandths of a radian,
which it is, and that strong baseline is the single most important correctness gate
in the whole project, because a weak baseline would make every later comparison
meaningless and a reviewer would catch it in a second.

The middle layer is the learned residual, and it is kept linear in its weights on
purpose. The residual is W transpose times phi of x, where phi is a fixed bank of
radial basis functions over the robot's own joint position and velocity, drawn once
from a seeded random set of centers and then never moved, so only the weights
adapt. Keeping it linear in the weights is what makes the stability argument clean,
because the parameter error then enters the storage function quadratically and the
update law falls straight out of asking the storage function to decrease. The
weights ride a Lyapunov-derived law,

    Wdot = Gamma * phi(x) * s^T  -  sigma * Gamma * W,

where s equals e dot plus lambda times e is the composite error that folds position
and velocity error into one quantity, the first term drives the residual to cancel
the model error the drift introduced, and the second term is leakage, an
e-modification that bleeds the weights back toward zero whenever the motion is not
exciting the regressor, so the estimate cannot wander off during the quiet stretches
when there is nothing to learn. With the Lyapunov function

    V = 1/2 s^2 + 1/2 trace(Wtilde^T Gamma^-1 Wtilde),

the derivative works out to V dot at most minus K s squared plus order sigma plus
order of the function-approximation error, which is uniform ultimate boundedness of
both the tracking error and the parameter error, so both stay inside a ball whose
size you set with the gains rather than growing without limit. The clean bound holds
in this linear-in-weights regime, and a deeper network breaks the clean proof, which
is a stated limitation and a future direction rather than something swept under the
rug. A note on signs, because they trip people up, the textbook law is written with
the tracking error as measured minus reference, which puts a minus on the first
term, and the baseline here defines the error as reference minus measured, the usual
convention for a PD command, so the identical stabilizing law shows up with a plus
on the first term, and nothing about the stability changes.

The top layer is safety, and its whole job is to keep the learned part inside the
regime where the bound is valid. It saturates the residual torque so the learned
term can never command more than a set magnitude, which bounds its authority over
the plant, and it projects the weight matrix back onto a ball of fixed radius after
every update, so even if the regressor goes quiet and leakage alone is slow, the
estimate cannot leave the certified set. The point of having safety as its own
switchable layer is that you can turn it off and measure exactly what it was buying,
which is how it earns its place on a number rather than by argument.

The self-observation feed is the only thing the self-model is allowed to see, and
it is built from what a real machine actually measures about itself, the joint
position and velocity and a finite-difference acceleration, the commanded torque, a
motor-current proxy that is the motor torque over the torque constant with the gear
correctly in the denominator, a back-EMF proxy from speed, an electrical-power
estimate, a lumped winding temperature from a first-order i squared R thermal model,
and an effort residual that is the applied torque minus what the frozen nominal
model says that torque should have been. That last channel is the tell, because when
the plant drifts the nominal model stops predicting the torque correctly and the gap
opens up there, and the drift is injected only into the true plant and never told to
the controller, so anything the self-model recovers it recovers from the robot's own
data.

## Testbeds and drift

Three small self-contained MuJoCo systems are reimplemented directly so the whole
thing is reproducible with no external reinforcement-learning stack, simplest first.
The 1-DOF pendulum is the cleanest place to show bounded online adaptation because
the dynamics are one equation. The 2-link reacher arm in a vertical plane is the
robotics-native case, where gravity loads both joints in a way that swings with
configuration, so a drifted payload corrupts the cancellation differently at every
pose. The planar arm pushing a free cube on a frictional floor is the most visually
legible, because the cost of getting the arm's own dynamics wrong shows up as the
cube ending somewhere it should not. Drift is injected by scaling the true plant's
payload mass, joint friction and damping, and an actuator torque constant that sags,
and the gradual case couples part of that to the winding temperature, so heating
from sustained current is what actually drives the wear, which is closer to how
hardware degrades over a long run than a scripted ramp on a clock.

## What the runs show

The no-drift baseline tracks tightly on all three systems, with RMS tracking error
of 1.30e-3 radians on the pendulum, 1.78e-3 on the reacher, and 3.60e-3 on the push
arm, all in the low thousandths of a radian, so the gate that everything else
depends on is genuinely met. Adding the adaptive residual with safety does not hurt
the clean case, it slightly helps it, because the residual also mops up the small
nominal friction mismatch.

Under a sudden mid-run shift, a doubled payload with stiffer friction and a torque
sag injected at a fixed time, the baseline takes a permanent hit while the adaptive
controller recovers within the run. On the pendulum the RMS tracking error drops
from 1.33e-2 for the baseline to 4.5e-4 for the safety-bounded adaptive controller,
which is about a thirty-fold reduction, on the reacher it goes from 1.29e-2 to
1.23e-3, and on the push arm from 6.5e-3 to 9.3e-4. Under the long thermal-coupled
gradual degradation the same pattern holds, with the pendulum improving from 5.5e-3
to 2.2e-4 and the reacher from 6.4e-3 to 9.8e-4, so the controller is self-adapting
online with no reset as its own parameters drift, which is the headline
self-modeling-under-wear result.

The offline protocol asks the question even more directly, because it takes a fixed
log of the self-observation recorded while the plant drifted under the plain
baseline, with no learning running so the effort residual is a clean readout of the
drift, fits the self-model on the early low-drift part of the run, and tests it on
the later high-drift part it never saw. On the pendulum the held-out prediction
error is 3.3e-2 newton meters against 2.7e-1 for simply assuming nothing changed,
which is roughly eight times better and an extrapolation R squared of 0.99, so the
robot really did learn its own drift from its own data well enough to forecast a
drift level it was never trained on. On the reacher the held-out error is 1.4e-1
against 5.0e-1 for assuming no drift, so it still captures real structure and beats
the do-nothing predictor by about three and a half times, but the R squared against
the held-out mean is negative because a static feature map fit on mild friction
under-predicts the magnitude once the friction has roughly tripled, which is an
honest limit of extrapolating this far with fixed features.

## Where it does not help, with equal weight

Two negatives matter as much as the wins. Under a low-excitation step-and-hold
reference the residual has too little time per setpoint to pay back its transient,
so adaptation barely improves tracking over the baseline and on the pendulum it is
slightly worse, 9.4e-2 against 9.6e-2 radians, which is exactly the regime where
this kind of online correction is not worth its cost and you should not sell it
there. And the offline self-model fails outright on the push arm, with a held-out
error of 6.6e-2 against 4.6e-2 for assuming no drift, meaning it does worse than
doing nothing, because that arm's joints are gravity free and its effort residual is
dominated by contact impulses from the cube rather than a smooth function of its own
state, so there is little learnable drift signal to recover. The method earns its
keep on the gravity-loaded systems where drift shows up as a smooth, predictable
model error, and it should be claimed only there.

The same low-excitation case is where the safety layer stops being insurance you
never see pay out and becomes the thing holding the system together. The naive
ablation, which updates the same residual by plain gradient descent with no leakage
and no projection, tracks just as well as the bounded variant under a rich reference
and sometimes a hair better, because leakage trades a little accuracy for robustness
you do not need when the motion is exciting. Starve the excitation and that trade
reverses hard. The naive weight norm walks off to roughly 1.3e3 on the pendulum,
3.9e4 on the reacher, and 1.3e7 on the push arm, the residual torque chatters
violently, and on the contact-rich push arm the system diverges outright with tens
of thousands of ticks outside the sane operating range, while the safety-bounded
variant holds its weight norm at the projection radius, keeps the torque smooth, and
stays stable at comparable tracking. That contrast, measured on the weight norm and
the divergence count rather than asserted, is the core reason leakage and projection
are in the loop at all.

## Limitations

This is simulation only and says so everywhere, on small low-degree-of-freedom
systems, and the clean stability bound holds only in the linear-in-weights regime
with a fixed hand-chosen radial-basis feature map and scalar safety caps, so it is a
controlled study of a mechanism and not a hardware result. The drift model is a
deliberate caricature of wear with hand-set coefficients, the offline contact case
shows the self-model can fail when the residual is not a smooth function of state,
the extrapolation degrades when the drift moves far past what the fit ever saw, and
none of this has touched a real motor, where the electrical and thermal signals
would be measured and noisy in ways this does not fully capture.

## Future work

The natural next steps build outward from the same spine. A forward-predicting
online world model rather than a static residual would let the robot anticipate its
drift instead of only correcting it after the error appears. A physics-constrained
or port-Hamiltonian residual could add capacity while keeping a boundedness
guarantee, which is the way past the linear-in-weights ceiling without giving up the
proof. Richer sensing than joint state and a current proxy would carry more of the
drift, contact-rich manipulation where the contact itself is modeled rather than
rejected would address the case that fails here, and a transfer to real hardware,
where the electrical and thermal channels are measured rather than simulated, is the
test that would tell us whether any of this survives contact with a real machine.
