# Dexterous-Hand WAM-prior RL + Reachability Benchmark — Action TODO

## What this project is (read this when you lose the thread)
- I'm training a dexterous robot hand. The world model (WAM) only **suggests** a rough 1-second plan to explore around — it's a hint, not the controller, and not the source of truth. The **real physics sim is the boss**: it gives the true reward and decides what's physically possible. The policy learns split-second reactions inside the sim, and I randomize the sim's physics (friction, mass, …) so the policy generalizes.
- I'm publishing two things. **Main contribution = a benchmark**: use the physics sim to test whether a world model's prediction is even physically achievable (nobody's done this). The **training method = the application** that proves the benchmark is useful.
- Two things never to forget: ① get a basic dexterous task + RL running first, and don't bet the whole project on the WAM being good; ② the "search the sim for an action that reproduces a predicted trajectory" engine is **the same for both the benchmark and the training method — build it once, use it twice**.

---

## Phase 0 — Foundation (do this first)
- [ ] Get a single dexterous-hand task + PPO baseline running in a contact-accurate sim (mjwarp / Isaac Lab); reach a usable success rate
- [ ] Decide the sim: MVP uses mjwarp's contact (perturb its contact params directly); bring in your own separating-plane sim later, when you need finer / more principled contact-error control
- [ ] Confirm you can read ground truth out of the sim: contact events, penetration depth, conserved quantities, object pose

## Phase 1 — WAM (your #1 · highest risk · de-risk early)
- [ ] Verify the WAM can produce a 1s trajectory **+ corresponding actions** starting from a **sim-rendered frame** (or does it only output video, needing an IDM bolted on?)
- [ ] Pick which WAM: DreamZero-style / Cosmos action-conditioned / DexWM-style latent
- [ ] Build the seed-from-config pipeline: sim state → render → WAM input; handle the **visual domain gap**
- [ ] Judge early how bad the 1s dexterous imagination is → if the WAM can't serve as a prior, **shift weight to the benchmark leg** (a bad WAM = low realizability, which is itself a result, not fatal)

## Phase 2 — Benchmark / Reachability certificate (main contribution · machinery shared with the method)
- [ ] **Feasibility certificate**: run MPPI/CEM in the sim to search for an action realizing τ_WAM; found = feasible
- [ ] **Infeasibility certificate**: physics-violation checks — interpenetration / teleportation / energy gain — as proof that "no action can realize this"
- [ ] Honestly report the **semi-decidable gray zone** (no realizing action found AND no violation certificate) as its own bucket
- [ ] **Perception / lifting layer**: lift WAM output to physical quantities comparable with sim state, plus tolerance bands (occlusion in dexterous is the hard part)
- [ ] Metric: realizability rate, broken down by feasible / infeasible / gray + the **type** of infeasibility

## Phase 3 — Method / sim-grounded WAM-prior RL (application · your #1 + #2 feed in here)
- [ ] Proposal mechanism: WAM provides the sampling mean to guide PPO/MPPI exploration (frame it as **exploration-variance reduction**)
- [ ] **Correctness anchor**: reward = **true sim return**, **not** matching the WAM
- [ ] Add a **feasibility filter** on WAM targets: only chase feasible ones; fall back to true-return RL for the infeasible ones
- [ ] **Your #2 — physical DR**: randomize friction / mass / contact stiffness / restitution / latency (this is your differentiator over WM-in-the-loop)
- [ ] Optional hierarchical: WAM = subgoal proposer, sim-RL = reactive tracker

## Phase 4 — Evaluation / headline results
- [ ] Sample efficiency: how much faster is WAM-prior + sim-grounded RL vs. plain PPO (proves the variance-reduction claim)
- [ ] Generalization: DR-trained policy vs. WM-in-the-loop baselines — which transfers / generalizes better
- [ ] **The key combined result**: does the WAM's realizability rate **predict downstream training acceleration / generalization** (your physics-grounded cheap-proxy ↔ downstream, the counterpart to DreamGen Bench)
- [ ] Head-to-heads: vs. EVA (learned IDM reward), vs. DexWM (CEM planning), vs. WMPO (RL inside imagination)
- [ ] seed-from-config ablation (the unexplored knob)

## Phase 5 — Cross-cutting / strategy
- [ ] **Read the closest prior art first**, to fix positioning and avoid reinventing:
  - [ ] EVA (2603.17808) — the most recent "executability gap," but uses a learned IDM reward
  - [ ] DexWM (2512.13644) — the most recent dexterous WM, but latent + CEM planning
  - [ ] WAV (2604.01985) — feasibility / reachability verifier, but fully learned
  - [ ] WMPO (2511.09515) / World-Env (2509.24948) — WM-in-the-loop RL (your inverse)
  - [ ] VP² (2304.13723) — sim-in-benchmark precedent, but only measures downstream success
  - [ ] DreamGen Bench (2505.12705) — generic video quality ↔ policy success (your contrast target)
- [ ] Staging: **submit the benchmark as the main contribution, the method as the application that validates it**
- [ ] Sweep arXiv again before submission (this area moves weekly)

---

### Key risks & reminders
- **The WAM is probably bad at dexterous contact** → don't bet everything on Phase 1; the benchmark leg is immune to this.
- **Correctness comes from the real sim, not the WAM** → reward = true return at all times; the WAM only shapes "where to look."
- **Physical DR is one of your moats** → scope it to "controllable physical-dynamics randomization" (concede visual diversity to WMs).
- **Reuse the machinery** → Phase 2's reachability search = Phase 3's proposal search; it's the same engine.
