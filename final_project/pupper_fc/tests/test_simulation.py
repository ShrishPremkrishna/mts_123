"""Physics and sensing seams, without robot access or external dependencies."""
import math
import unittest

from final_project.pupper_fc.model import Ball, Command, Field, Velocity
from final_project.pupper_fc.simulation import SCENARIOS, Simulator


class SimulationTests(unittest.TestCase):
    def advance(self, sim, seconds, vx=0.0, vy=0.0, wz=0.0):
        for _ in range(round(seconds / 0.05)):
            sim.step(Command(vx, vy, wz, expires_at=sim.time + 0.1))

    def score(self, sim, direction):
        sim.move_robot(0, 0.9)
        sim.move_ball(direction * (sim.field.length / 2 - sim.field.ball_radius - 0.01), 0)
        sim.nudge_ball(direction * 1.0, 0)
        self.advance(sim, 0.5)

    def test_seeded_rollout_and_observations_are_deterministic(self):
        first, second = Simulator(seed=19), Simulator(seed=19)
        for _ in range(30):
            for sim in (first, second):
                sim.step(Command(0.5, 0.1, 0.15, sim.time + 0.1))
            self.assertEqual(first.observe(delay=0.1, noise=0.02),
                             second.observe(delay=0.1, noise=0.02))
        self.assertEqual(first.snapshot(), second.snapshot())
        self.assertNotEqual(Simulator(seed=20).observe(noise=0.02), Simulator(seed=19).observe(noise=0.02))

    def test_delay_freeze_and_loss_do_not_leak_new_state(self):
        sim = Simulator(scenario="moving_ball")
        first = sim.observe()
        self.assertEqual(first.ball.vx, 0)
        self.advance(sim, 0.2)
        delayed = sim.observe(delay=0.15)
        self.assertAlmostEqual(delayed.captured_at, 0.05)
        self.assertLess(delayed.ball.x, sim.ball.x)
        self.assertGreater(delayed.ball.vx, 0)
        self.assertNotEqual(delayed.ball.vx, sim.ball.vx)
        self.advance(sim, 0.1)
        self.assertIs(sim.observe(frozen=True), delayed)
        current = sim.observe()
        self.assertAlmostEqual(current.captured_at, 0.3)
        self.assertIs(sim.observe(), current)
        self.assertIsNone(sim.observe(ball_visible=False).ball)
        self.assertEqual(sim.observe().ball.vx, 0)

    def test_goal_and_fall_signals_follow_frame_capture_time(self):
        sim = Simulator()
        sim.observe()
        sim.fallen = True
        self.advance(sim, 0.05)
        self.assertFalse(sim.observe(delay=0.05).fallen)
        self.assertTrue(sim.observe().fallen)
        sim.fallen = False
        self.score(sim, 1)
        self.assertEqual(sim.observe(delay=100).goal, 0)
        self.assertEqual(sim.observe().goal, 1)

    def test_same_delayed_capture_retains_noise_while_world_advances(self):
        sim = Simulator(scenario="moving_ball")
        first = sim.observe(delay=0.2, noise=0.04)
        self.advance(sim, 0.1)
        self.assertIs(sim.observe(delay=0.2, noise=0.04), first)
        self.advance(sim, 0.1)
        self.assertIs(sim.observe(delay=0.2, noise=0.04), first)
        self.advance(sim, 0.05)
        self.assertNotEqual(sim.observe(delay=0.2, noise=0.04).captured_at, first.captured_at)

    def test_full_ball_must_cross_goal_line(self):
        sim = Simulator()
        sim.move_robot(0, 0.9)
        sim.move_ball(sim.field.length / 2 - sim.field.ball_radius, 0)
        sim.nudge_ball(1, 0)
        sim.step(Command(), dt=0.1)
        self.assertGreater(sim.ball.x, sim.field.length / 2)
        self.assertLess(sim.ball.x, sim.field.length / 2 + sim.field.ball_radius)
        self.assertEqual(sim.goal, 0)
        sim.step(Command(), dt=0.1)
        self.assertEqual(sim.goal, 1)
        scored_ball = sim.ball
        sim.nudge_ball(-3, 0)
        self.advance(sim, 1)
        self.assertEqual(sim.ball, scored_ball)

    def test_both_goals_and_mirrored_attack_direction(self):
        for attack_sign in (-1, 1):
            for direction in (-1, 1):
                with self.subTest(attack_sign=attack_sign, direction=direction):
                    sim = Simulator(field=Field(attack_sign=attack_sign))
                    self.score(sim, direction)
                    self.assertEqual(sim.goal, attack_sign * direction)
                    self.assertEqual((sim.ball.vx, sim.ball.vy), (0, 0))

    def test_after_goal_robot_can_return_home_and_ball_placement_retains_goal(self):
        sim = Simulator()
        self.score(sim, 1)
        before = sim.robot.x
        self.advance(sim, 0.5, vx=-0.5)
        self.assertLess(sim.robot.x, before)
        sim.move_ball(0, 0)
        self.assertEqual(sim.goal, 1)
        self.advance(sim, 0.1)
        self.assertEqual(sim.ball, Ball(0, 0))
        sim.goal = 0
        sim.nudge_ball(0.5, 0)
        self.advance(sim, 0.1)
        self.assertGreater(sim.ball.x, 0)

    def test_walls_bounce_outside_mouth(self):
        sim = Simulator()
        sim.move_robot(0, -0.8)
        sim.move_ball(sim.field.length / 2 - sim.field.ball_radius - 0.01, 0.8)
        sim.nudge_ball(1, 0)
        self.advance(sim, 0.2)
        self.assertEqual(sim.goal, 0)
        self.assertLess(sim.ball.vx, 0)
        sim.move_ball(0.5, sim.field.width / 2 - sim.field.ball_radius - 0.01)
        sim.nudge_ball(0, 1)
        self.advance(sim, 0.2)
        self.assertLess(sim.ball.vy, 0)

    def test_restart_play_discards_prior_goal_packets_but_preserves_robot_motion(self):
        sim = Simulator()
        self.score(sim, 1)
        goal_packet = sim.observe()
        self.assertEqual(goal_packet.goal, 1)
        self.advance(sim, 0.1, vx=-0.3)
        clock, robot, velocity = sim.time, sim.robot, sim.velocity
        sim.restart_play()
        self.assertEqual((sim.time, sim.robot, sim.velocity), (clock, robot, velocity))
        restarted = sim.observe(delay=100, frozen=True)
        self.assertEqual(restarted.goal, 0)
        self.assertEqual(restarted.captured_at, clock)
        self.assertEqual(restarted.ball, Ball(0, 0))
        self.assertIs(sim.observe(frozen=True), restarted)
        self.assertEqual(sim.goal, 0)

    def test_invalid_restart_leaves_goal_and_packet_intact(self):
        sim = Simulator()
        self.score(sim, 1)
        packet = sim.observe()
        with self.assertRaises(ValueError):
            sim.restart_play(math.nan, 0)
        self.assertEqual(sim.goal, 1)
        self.assertIs(sim.observe(frozen=True), packet)

    def test_fall_api_refreshes_current_frame_without_changing_delayed_or_frozen_frames(self):
        sim = Simulator()
        initial = sim.observe()
        self.advance(sim, 0.1)
        sim.set_fallen(True)
        self.assertIs(sim.observe(frozen=True), initial)
        self.assertFalse(sim.observe(delay=0.1).fallen)
        self.assertTrue(sim.observe().fallen)
        with self.assertRaises(ValueError):
            sim.set_fallen(1)

    def test_stop_brakes_with_bounded_drift(self):
        sim = Simulator()
        sim.move_robot(-1.2, -0.8, 0)
        self.advance(sim, 0.8, vx=0.65)
        before, speed = sim.robot.x, sim.velocity.vx
        sim.step(Command(expires_at=sim.time + 1))
        self.assertGreater(sim.robot.x, before)
        self.assertAlmostEqual(speed - sim.velocity.vx, sim.limits.acceleration * 0.05)
        self.advance(sim, 1)
        self.assertAlmostEqual(sim.velocity.vx, 0)
        self.assertLessEqual(sim.robot.x - before, speed * speed / (2 * sim.limits.acceleration) + 0.001)

    def test_command_expiration_wrong_identity_and_nonfinite_brake(self):
        for bad in (Command(0.65, expires_at=0),
                    Command(0.65, expires_at=100, robot_id="other"),
                    Command(math.nan, expires_at=100),
                    Command(0.65, expires_at=math.inf)):
            with self.subTest(command=bad):
                sim = Simulator()
                before = sim.robot
                sim.step(bad)
                self.assertEqual(sim.robot, before)
                self.assertEqual(sim.velocity, Velocity())
        sim = Simulator()
        sim.move_robot(-1.2, -0.8)
        self.advance(sim, 0.3, vx=0.65)
        old = sim.velocity.vx
        sim.step(Command(0.65, expires_at=sim.time + 0.025))
        self.assertAlmostEqual(sim.velocity.vx, old, places=10)
        sim.step(Command(math.nan, expires_at=100))
        self.assertLess(sim.velocity.vx, old)

    def test_body_velocity_yaw_and_bounds(self):
        sim = Simulator()
        sim.move_robot(-0.8, -0.8, math.pi / 2)
        self.advance(sim, 0.5, vx=100)
        self.assertAlmostEqual(sim.robot.x, -0.8)
        self.assertGreater(sim.robot.y, -0.8)
        self.advance(sim, 5, vx=100)
        self.assertLessEqual(sim.robot.y, sim.field.width / 2 - sim.field.robot_radius)
        self.assertLessEqual(abs(sim.velocity.vx), sim.limits.vx + 1e-9)

    def test_malformed_command_fields_brake_without_raising_or_coercion(self):
        invalid = (None, "1", True, False, 1j, [], {}, math.nan, math.inf, 10 ** 400)
        for field in ("vx", "vy", "wz", "expires_at"):
            for value in invalid:
                with self.subTest(field=field, value=value):
                    sim = Simulator()
                    sim.move_robot(-1.2, -0.8)
                    self.advance(sim, 0.3, vx=0.65)
                    before, speed = sim.robot.x, sim.velocity.vx
                    values = dict(vx=0.65, expires_at=100)
                    values[field] = value
                    sim.step(Command(**values))
                    self.assertAlmostEqual(sim.velocity.vx, speed - sim.limits.acceleration * 0.05)
                    self.assertGreater(sim.robot.x, before)
                    self.assertEqual((sim.velocity.vy, sim.velocity.wz), (0, 0))

    def test_fallen_robot_does_not_travel(self):
        sim = Simulator()
        self.advance(sim, 0.3, vx=0.65)
        sim.fallen = True
        before = sim.robot
        self.advance(sim, 1, vx=0.65, vy=0.2, wz=1)
        self.assertEqual(sim.robot, before)
        self.assertEqual(sim.velocity, Velocity())

    def test_sliding_against_wall_preserves_body_speed_limits(self):
        sim = Simulator()
        sim.move_robot(sim.field.length / 2 - sim.field.robot_radius - 0.01, 0.1, math.pi / 4)
        for _ in range(60):
            sim.step(Command(100, 100, 0, sim.time + 0.1))
            self.assertLessEqual(abs(sim.velocity.vx), sim.limits.vx + 1e-9)
            self.assertLessEqual(abs(sim.velocity.vy), sim.limits.vy + 1e-9)

    def test_distant_robot_cannot_kick_and_stationary_contact_cannot_add_speed(self):
        sim = Simulator()
        sim.move_robot(-1, -0.7)
        initial = sim.ball
        self.advance(sim, 1, vx=0.65)
        self.assertEqual(sim.ball, initial)
        sim.move_robot(0, 0)
        sim.move_ball(0.2, 0)
        self.advance(sim, 0.1)
        self.assertEqual((sim.ball.vx, sim.ball.vy), (0, 0))
        self.assertGreaterEqual(sim.ball.x, sim.field.bumper_offset + sim.field.ball_radius)

    def test_frontal_run_through_can_score_in_both_directions(self):
        for sign in (-1, 1):
            sim = Simulator(field=Field(attack_sign=sign))
            self.advance(sim, 5, vx=0.65)
            self.assertEqual(sim.goal, 1)

    def test_placement_is_bounded_and_refreshes_current_observation(self):
        sim = Simulator()
        previous = sim.observe()
        sim.move_ball(100, 100)
        sim.move_robot(-100, -100, 11 * math.pi)
        packet = sim.observe()
        self.assertEqual(packet.captured_at, previous.captured_at)
        self.assertNotEqual(packet.ball, previous.ball)
        self.assertLessEqual(abs(sim.ball.x), sim.field.length / 2 - sim.field.ball_radius)
        self.assertLessEqual(abs(sim.robot.y), sim.field.width / 2 - sim.field.robot_radius)
        self.assertLessEqual(abs(sim.robot.yaw), math.pi)

    def test_validation_and_all_scenarios(self):
        for scenario in SCENARIOS:
            sim = Simulator(scenario=scenario)
            self.assertEqual(sim.snapshot()["scenario"], scenario)
            sim.step(Command())
        for dt in (0, -0.1, 0.21, math.nan, math.inf):
            with self.assertRaises(ValueError):
                Simulator().step(Command(), dt)
        for kwargs in ({"scenario": "missing"}, {"seed": None}, {"field": Field(attack_sign=0)}):
            with self.assertRaises(ValueError):
                Simulator(**kwargs)
        for method, args in (("move_ball", (math.nan, 0)), ("move_robot", (0, math.inf)),
                             ("nudge_ball", (math.inf, 0)), ("observe", (-1,))):
            with self.assertRaises(ValueError):
                getattr(Simulator(), method)(*args)


if __name__ == "__main__":
    unittest.main()
