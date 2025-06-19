from collections import namedtuple
import time
from typing import (Any, ClassVar, Dict, Final, List, Mapping, Optional,
                    Sequence)

from typing_extensions import Self
from viam.components.generic import *
from viam.components.sensor import Sensor
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import Geometry, ResourceName
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.components.board import Board
from viam.utils import ValueTypes, SensorReading
from viam.logging import getLogger
import time
import asyncio

MAX_CHANGES = 67

Protocol = namedtuple('Protocol',
                      ['pulselength',
                       'sync_high', 'sync_low',
                       'zero_high', 'zero_low',
                       'one_high', 'one_low'])
PROTOCOLS = (None,
             Protocol(350, 1, 31, 1, 3, 3, 1),
             Protocol(650, 1, 10, 1, 2, 2, 1),
             Protocol(100, 30, 71, 4, 11, 9, 6),
             Protocol(380, 1, 6, 1, 3, 3, 1),
             Protocol(500, 6, 14, 1, 2, 2, 1),
             Protocol(200, 1, 10, 1, 5, 1, 1))


class Receiver(Sensor, EasyResource):
    # To enable debug-level logging, either run viam-server with the --debug option,
    # or configure your resource/machine to display debug logs.
    MODEL: ClassVar[Model] = Model(ModelFamily("grant-dev", "433mhz-rf"), "receiver")

    board: Board
    data_pin: str

    rx_tolerance = 0.8
    rx_timings = [0] * (MAX_CHANGES + 1)
    rx_last_timestamp = 0
    rx_change_count = 0
    rx_repeat_count = 0
    rx_pulselength = 3500 # microseconds
    
    rx_code = 0
    rx_code_timestamp = None

    stream_task = None
    _stop_event = asyncio.Event()
    
    
    @classmethod
    def new(
        cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> Self:
        """This method creates a new instance of this Generic component.
        The default implementation sets the name from the `config` parameter and then calls `reconfigure`.

        Args:
            config (ComponentConfig): The configuration for this resource
            dependencies (Mapping[ResourceName, ResourceBase]): The dependencies (both implicit and explicit)

        Returns:
            Self: The resource
        """
        cls.logger = getLogger(config.name)
        return super().new(config, dependencies)

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Sequence[str]:
        """This method allows you to validate the configuration object received from the machine,
        as well as to return any implicit dependencies based on that `config`.

        Args:
            config (ComponentConfig): The configuration for this resource

        Returns:
            Sequence[str]: A list of implicit dependencies
        """

        attrs = config.attributes.fields
        if "board" not in attrs:
            raise Exception("board is required")
        if "data_pin" not in attrs:
            raise Exception("data_pin is required")
        
        board_name = attrs["board"].string_value
        return [board_name]

    def reconfigure(
        self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ):
        """This method allows you to dynamically update your service when it receives a new `config` object.

        Args:
            config (ComponentConfig): The new configuration
            dependencies (Mapping[ResourceName, ResourceBase]): Any dependencies (both implicit and explicit)
        """
        if self.stream_task is not None:
            self._stop_stream_task()

        attrs = config.attributes.fields
        board_name = attrs["board"].string_value
        self.board = dependencies[Board.get_resource_name(board_name)]
        self.data_pin = attrs["data_pin"].string_value
        self.gpio = self.board.gpio_pin_by_name(self.data_pin)
        if "expected_pulse_length" in attrs:
            self.rx_pulselength = int(attrs["expected_pulse_length"].number_value)
        if "noise_tolerance" in attrs:
            self.rx_tolerance = attrs["noise_tolerance"].number_value

        self._stop_event.clear()
        self.stream_task = asyncio.create_task(self._process_stream_ticks())
        return super().reconfigure(config, dependencies)
    
    def _stop_stream_task(self):
        self._stop_event.set()
        if self.stream_task is not None:
            self.stream_task.cancel()
            self.stream_task = None
    
    async def _process_stream_ticks(self):
        try:
            data_interrupt = await self.board.digital_interrupt_by_name(name=self.data_pin)
            rising_edge_ts = -1
            falling_edge_ts = -1
            last_pulse_start_ts = -1
            last_pulse_end_ts = -1
            code = 0
            async for tick in await self.board.stream_ticks([data_interrupt]):
                if self._stop_event.is_set():
                    break
                # self.logger.info(f"Tick: {tick.time / 1000} microseconds {tick.high}")
                if tick.high:
                    rising_edge_ts = tick.time
                else:
                    falling_edge_ts = tick.time
                    
                if falling_edge_ts > rising_edge_ts:
                    # pulse has ended
                    high_duration = falling_edge_ts - rising_edge_ts
                    high_duration_us = high_duration / 1000
                    if high_duration_us < self.rx_pulselength * self.rx_tolerance:
                        # filter out very short pulses as noise
                        high_duration = -1
                        rising_edge_ts = -1
                        falling_edge_ts = -1
                        continue
                    
                    # self.logger.info(f"Wave width: {high_duration / 1000} microseconds")
                    
                
                    distance_from_last_pulse_end_us = (rising_edge_ts - last_pulse_end_ts) / 1000

                    if last_pulse_end_ts != -1:

                        last_pulse_duration_us = (last_pulse_end_ts - last_pulse_start_ts) / 1000

                        self.logger.info(f"Last pulse duration: {last_pulse_duration_us} us, distance from last pulse {distance_from_last_pulse_end_us} us")
                        if distance_from_last_pulse_end_us > 6 * self.rx_pulselength:
                            # sync signal, set the reading
                            self.rx_code = code
                            self.rx_code_timestamp = last_pulse_end_ts
                            self.logger.info(f"Sync signal, final code: {format(code, f'#024b')}")
                            code = 0
                        else:
                            # data signal

                            # short high followed by long low is a 0
                            if last_pulse_duration_us < 2 * self.rx_pulselength and distance_from_last_pulse_end_us > 2 * self.rx_pulselength:
                                code <<= 1
                                self.logger.info(f"Signal 0, Binary code: {format(code, f'#024b')}")

                            # long high followed by short low is a 1
                            if last_pulse_duration_us > 2 * self.rx_pulselength and distance_from_last_pulse_end_us < 2 * self.rx_pulselength:
                                code <<= 1
                                code |= 1
                                self.logger.info(f"Signal 1, Binary code: {format(code, f'#024b')}")
                    
                    last_pulse_start_ts = rising_edge_ts
                    last_pulse_end_ts = falling_edge_ts
        except asyncio.CancelledError:
            pass
        except Exception as e:
            self.logger.error(f"Error streaming ticks: {e}")
            
    async def get_readings(
        self,
        *,
        timeout: Optional[float] = None,
        **kwargs
    ) -> Mapping[str, SensorReading]:
        return {"code": str(self.rx_code)}

    async def close(self):
        self._stop_stream_task()

    async def do_command(
        self,
        command: Mapping[str, ValueTypes],
        *,
        timeout: Optional[float] = None,
        **kwargs
    ) -> Mapping[str, ValueTypes]:
        
        raise NotImplementedError()

    async def get_geometries(
        self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None
    ) -> List[Geometry]:
        self.logger.error("`get_geometries` is not implemented")
        raise NotImplementedError()

