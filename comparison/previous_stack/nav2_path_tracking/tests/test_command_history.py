"""验证停步时保留旧指令，避免历史被最新零速度瞬间覆盖。"""
import unittest
from forest_nav_control.command_history import CommandHistory


class CommandHistoryTests(unittest.TestCase):
    def test_stop_retains_seven_real_previous_frames(self):
        history = CommandHistory()
        moving = (.4, 0., .2)
        self.assertEqual(history.append(moving, 24), moving * 8)
        stopped = history.append((0., 0., 0.), 24)
        self.assertEqual(stopped, moving * 7 + (0., 0., 0.))
        for _ in range(7):
            stopped = history.append((0., 0., 0.), 24)
        self.assertEqual(stopped, (0.,) * 24)

    def test_order_and_reset(self):
        history = CommandHistory()
        history.append((0., 0., 0.), 6)
        history.append((.1, 0., 0.), 6)
        self.assertEqual(history.append((.2, 0., -.1), 6), (.1, 0., 0., .2, 0., -.1))
        history.reset()
        self.assertEqual(history.append((.3, 0., 0.), 6), (.3, 0., 0.) * 2)

    def test_invalid_contract_rejected(self):
        history = CommandHistory()
        for width in (0, -3, 5):
            with self.assertRaises(ValueError): history.append((0, 0, 0), width)
        with self.assertRaises(ValueError): history.append((float('nan'), 0, 0), 24)
        history.append((0, 0, 0), 24)
        with self.assertRaises(ValueError): history.append((0, 0, 0), 12)


if __name__ == '__main__': unittest.main()
