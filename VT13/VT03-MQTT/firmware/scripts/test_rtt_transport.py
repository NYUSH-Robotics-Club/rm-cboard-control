"""Check RTT target selection and attach failures with simulated probe sessions.

These tests never connect to a board or consume a hardware RTT channel.
"""
import unittest
from unittest.mock import Mock, patch

from pyocd.core.exceptions import TargetSupportError
from pyocd.core.target import Target

from scripts.rtt_common.rtt_transport import RTTTransport, RTTTransportConfig


class RTTTransportTests(unittest.TestCase):
    def setUp(self):
        self.transport = RTTTransport(RTTTransportConfig(backend="pyocd"))
        self.session = Mock()
        self.session.target.get_state.return_value = Target.State.RUNNING
        self.channel = Mock()
        self.channel.name = "dashboard"
        self.rtt = Mock(up_channels=[self.channel])
        self.choose = patch(
            "pyocd.core.helpers.ConnectHelper.session_with_chosen_probe",
            return_value=self.session,
        ).start()
        self.find = patch(
            "pyocd.debug.rtt.RTTControlBlock.from_target", return_value=self.rtt,
        ).start()
        self.addCleanup(patch.stopall)

    def test_cboard_normal_attach_preserves_execution_state(self):
        self.transport.open()
        kwargs = self.choose.call_args.kwargs
        self.assertEqual(kwargs["target_override"], "stm32f407ighx")
        self.assertEqual(kwargs["options"]["frequency"], 4000000)
        self.assertEqual(kwargs["options"]["connect_mode"], "attach")
        self.assertFalse(kwargs["options"]["resume_on_disconnect"])
        self.assertFalse(kwargs["options"]["auto_unlock"])
        self.assertIn("DebugCoreStart", kwargs["options"]["pack.debug_sequences.disabled_sequences"])
        self.assertEqual(self.transport.available_channels(), ["0:dashboard"])
        self.transport.close()
        self.session.close.assert_called_once()
        self.session.target.resume.assert_not_called()
        self.session.target.halt.assert_not_called()
        self.session.target.reset.assert_not_called()

    def test_explicit_other_target_is_not_replaced(self):
        self.transport.config.device = "custom_mcu"
        self.transport.open()
        self.assertEqual(self.choose.call_args.kwargs["target_override"], "custom_mcu")
        self.transport.close()

    def test_missing_pack_reports_install_command_before_open(self):
        self.choose.side_effect = TargetSupportError("missing target")
        with self.assertRaisesRegex(RuntimeError, "python -m pyocd pack install stm32f407ighx"):
            self.transport.open()
        self.session.open.assert_not_called()
        self.find.assert_not_called()

    def test_halted_board_is_not_resumed_or_read(self):
        self.session.target.get_state.return_value = Target.State.HALTED
        with self.assertRaisesRegex(RuntimeError, "Target is halted"):
            self.transport.open()
        self.session.target.resume.assert_not_called()
        self.session.close.assert_called_once()
        self.find.assert_not_called()

    def test_rtt_failure_closes_session(self):
        self.rtt.start.side_effect = RuntimeError("no RTT control block")
        with self.assertRaisesRegex(RuntimeError, "no RTT control block"):
            self.transport.open()
        self.session.close.assert_called_once()
        self.assertIsNone(self.transport._session)

    def test_explicit_halt_and_reset_modes_keep_restart_behavior(self):
        for mode in ("halt", "under-reset"):
            with self.subTest(mode=mode):
                self.transport.config.connect_mode = mode
                self.session.target.get_state.return_value = Target.State.HALTED
                self.session.target.resume.reset_mock()
                self.transport.open()
                self.assertEqual(self.choose.call_args.kwargs["options"]["connect_mode"], mode)
                self.session.target.resume.assert_called_once()
                self.transport.close()

    def test_no_probe_reports_stlink_as_supported(self):
        self.choose.return_value = None
        with self.assertRaisesRegex(RuntimeError, "No ST-Link"):
            self.transport.open()

    def test_auto_does_not_mask_pyocd_error_with_missing_jlink(self):
        self.transport.config.backend = "auto"
        with patch.object(self.transport, "_has_connected_jlink", return_value=False), \
                patch.object(self.transport, "_open_jlink") as jlink:
            self.choose.side_effect = RuntimeError("USB access denied")
            with self.assertRaisesRegex(RuntimeError, "USB access denied"):
                self.transport.open()
            jlink.assert_not_called()

    def test_visible_jlink_keeps_its_existing_device_name(self):
        self.transport.config.backend = "auto"
        with patch.object(self.transport, "_has_connected_jlink", return_value=True), \
                patch.object(self.transport, "_open_jlink") as jlink:
            self.transport.open()
            jlink.assert_called_once()
        self.assertEqual(self.transport.config.device, "STM32F407IG")
        self.assertEqual(self.transport.backend_in_use, "jlink")
        self.choose.assert_not_called()


if __name__ == "__main__":
    unittest.main()
