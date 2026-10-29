"""Regression tests using simulated transports; no hardware is opened."""

import importlib
import io
from types import SimpleNamespace
from unittest.mock import Mock, call, patch

import pytest
import serial

from teraffe_drive_control_agent.config.teraffe_config import AMT212_IDS
from teraffe_drive_control_agent.controllers.lift_controller import LiftController
from teraffe_drive_control_agent.hardware.encoder import read_position
from teraffe_drive_control_agent.modules.lift_module import LiftModule
from teraffe_drive_control_agent import robot_hardware_web_ui as web
from teraffe_drive_control_agent import robot_tension_web_ui as tension


def make_lift():
    """Build a lift with mock motors and an unavailable encoder."""
    return LiftModule('test', Mock(), Mock(), None, 10.0, 150.0)


@pytest.mark.parametrize('direction', ['extend', 'retract'])
def test_lift_rejects_motion_without_encoder(direction):
    """A lost encoder must stop both motors instead of starting a move."""
    lift = make_lift()
    assert getattr(lift, direction)(500) is False
    lift.lift_motor.stop.assert_called_once()
    lift.assist_motor.stop_assist.assert_called_once()
    lift.lift_motor.set_erpm.assert_not_called()
    lift.assist_motor.assist.assert_not_called()


def test_lift_stops_when_encoder_is_lost_during_motion():
    """The periodic check must stop an already moving lift on read failure."""
    lift = make_lift()
    assert lift.check_and_stop_if_limit() is True
    lift.lift_motor.stop.assert_called_once()
    lift.assist_motor.stop_assist.assert_called_once()


@pytest.mark.parametrize('direction, angle', [('extend', 150), ('retract', 10)])
def test_lift_rejects_motion_at_directional_limit(direction, angle):
    """The existing travel limits must still block outward movement."""
    lift = make_lift()
    lift.get_position = Mock(return_value=angle)
    assert getattr(lift, direction)(500) is False
    lift.lift_motor.set_erpm.assert_not_called()


@pytest.mark.parametrize('direction, erpm', [('extend', 500), ('retract', -500)])
def test_lift_moves_with_valid_feedback(direction, erpm):
    """Valid feedback inside the travel range must allow motion."""
    lift = make_lift()
    lift.get_position = Mock(return_value=80)
    assert getattr(lift, direction)(500) is True
    lift.lift_motor.set_erpm.assert_called_once_with(erpm)


def test_encoder_restores_timeout_after_transport_error():
    """Failed reads must return no sample and restore serial settings."""
    transport = Mock(is_open=True, timeout=0.25)
    transport.read.side_effect = serial.SerialException('disconnected')
    assert read_position(transport, 0x1C) == (None, None)
    assert transport.timeout == 0.25
    assert transport.rts is True


def test_encoder_reads_selected_address_and_restores_timeout():
    """A valid zero-position checksum must decode using the selected leg."""
    transport = Mock(is_open=True, timeout=0.25)
    transport.read.return_value = b'\x00\xc0'
    assert read_position(transport, 0x2C) == (0, 0.0)
    transport.write.assert_called_once_with(b'\x2c')
    assert transport.timeout == 0.25


def test_controller_starts_with_missing_hardware():
    """Missing devices must not cause import or periodic status failures."""
    module = importlib.import_module(LiftController.__module__)
    bus = Mock()
    bus.open.side_effect = OSError('no CAN device')
    port = Mock()
    port.openPort.return_value = False
    with (
        patch.object(module.Node, '__init__', return_value=None),
        patch.object(LiftController, 'get_logger', return_value=Mock()),
        patch.object(LiftController, 'create_timer'),
        patch.object(module, 'CanBus', return_value=bus),
        patch.object(module, 'PortHandler', return_value=port),
        patch.object(module.serial, 'Serial', side_effect=serial.SerialException('no serial')),
    ):
        controller = LiftController()
        assert not any(controller.get_connection_status().values())
        for leg, address in AMT212_IDS.items():
            assert controller.get_lift(leg).encoder_addr == address
            assert controller.get_lift(leg).encoder_lock is controller.encoder_lock
            assert controller.get_leg_position(leg) is None
        controller.timer_callback()
        first, *others = controller.lift_modules.values()
        first.check_and_stop_if_limit = Mock(side_effect=serial.SerialException('lost'))
        for lift in others:
            lift.check_and_stop_if_limit = Mock(return_value=False)
        controller.timer_callback()
        for lift in others:
            lift.check_and_stop_if_limit.assert_called_once()


@pytest.mark.parametrize('fail_read', [False, True])
def test_web_serial_worker_zeros_current_before_close(fail_read):
    """Normal shutdown and a failed read must both clear applied current."""
    worker = web.SerialWorker(web.SharedState(), '/mock', 115200, 115200)
    port = Mock()
    port.openPort.return_value = True
    worker.set_baudrate = Mock()
    worker.set_current_mode = Mock()
    worker.write_goal_current = Mock()
    worker.read_dxl = Mock()

    def read_encoder(*args):
        worker.stop()
        if fail_read:
            raise serial.SerialException('disconnected')

    worker.read_encoder = read_encoder
    worker.enqueue('current', 'FL', 0.05)
    with patch.object(web, 'PortHandler', return_value=port):
        worker.run()
    assert worker.write_goal_current.call_args_list == [call(111, 0.05), call(111, 0.0)]
    port.closePort.assert_called_once()


@pytest.mark.parametrize('worker_type', ['serial', 'can'])
def test_unknown_module_is_rejected_before_queueing(worker_type):
    """Bad API input must not terminate a transport worker."""
    state = web.SharedState()
    if worker_type == 'serial':
        worker = web.SerialWorker(state, '/mock', 115200, 115200)
        with pytest.raises(ValueError):
            worker.enqueue('current', 'invalid', 0.05)
    else:
        worker = web.AkWorker(state, 'can0', 1000000)
        with pytest.raises(ValueError):
            worker.enqueue('invalid', 500)
    assert not worker.commands


def test_reconfigure_waits_for_old_workers_to_close():
    """A new serial owner must not start while an old worker still runs."""
    args = SimpleNamespace(
        serial_port='/mock', dxl_baudrate=115200, encoder_baudrate=115200,
        can_channel='can0', can_bitrate=1000000,
    )
    app = web.App(args)
    old_serial, old_can = Mock(), Mock()
    app.serial_worker, app.ak_worker = old_serial, old_can
    calls = Mock()
    calls.attach_mock(old_serial, 'serial')
    calls.attach_mock(old_can, 'can')
    with patch.object(web, 'SerialWorker') as new_serial, patch.object(web, 'AkWorker') as new_can:
        calls.attach_mock(new_serial, 'new_serial')
        app.reconfigure({'serial_port': '/new'})
    assert calls.mock_calls[:4] == [
        call.serial.stop(), call.can.stop(), call.serial.join(), call.can.join(),
    ]
    new_serial.return_value.start.assert_called_once()
    new_can.return_value.start.assert_called_once()


def test_can_worker_stops_motor_on_unexpected_receive_error():
    """Unexpected transport errors must still run the CAN shutdown path."""
    worker = web.AkWorker(web.SharedState(), 'can0', 1000000)
    bus = Mock()
    bus.recv.side_effect = OSError('USB disconnected')
    worker.enqueue('FL', 500)
    with (
        patch.object(web, 'setup_socketcan', return_value=(True, 'ready')),
        patch.object(web.os.path, 'exists', return_value=True),
        patch.object(web.can.interface, 'Bus', return_value=bus),
    ):
        worker.run()
    assert bus.send.call_args_list[0].args[0].data == bytes([0, 0, 1, 244])
    assert bus.send.call_args_list[-1].args[0].data == bytes(4)
    bus.shutdown.assert_called_once()


@pytest.mark.parametrize('module', [web, tension])
@pytest.mark.parametrize('body', [b'{', b'[]'])
def test_web_returns_bad_request_for_invalid_json(module, body):
    """Malformed API requests must get a response without touching hardware."""
    app = Mock()
    handler_type = module.make_handler(app)
    handler = handler_type.__new__(handler_type)
    handler.path = '/api/dxl/current'
    handler.headers = {'Content-Length': str(len(body))}
    handler.rfile = io.BytesIO(body)
    handler.respond_json = Mock()
    handler.do_POST()
    assert handler.respond_json.call_args.args[1] == 400
    app.serial_worker.enqueue.assert_not_called()


@pytest.mark.parametrize('module', [web, tension])
def test_busy_http_port_does_not_start_hardware(module):
    """An occupied HTTP port must fail before starting hardware workers."""
    with (
        patch.object(module, 'App') as app,
        patch.object(module, 'ThreadingHTTPServer', side_effect=OSError('port in use')),
    ):
        with pytest.raises(OSError):
            module.main([])
    app.return_value.start.assert_not_called()


def test_md_monitor_closes_server_without_shutdown_deadlock():
    """Ctrl+C must close the server without waiting on its own serve loop."""
    module = importlib.import_module('drive_steering_status_ui')
    with (
        patch.object(module, 'rclpy'),
        patch.object(module, 'MdStatusNode'),
        patch.object(module.threading, 'Thread'),
        patch.object(module, 'ThreadingHTTPServer') as server_type,
    ):
        server = server_type.return_value
        server.serve_forever.side_effect = KeyboardInterrupt
        module.main([])
    server.server_close.assert_called_once()
    server.shutdown.assert_not_called()
