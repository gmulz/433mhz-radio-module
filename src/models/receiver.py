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

    rx_tolerance = 80
    rx_timings = [0] * (MAX_CHANGES + 1)
    rx_last_timestamp = 0
    rx_change_count = 0
    rx_repeat_count = 0

    rx_code = 0
    rx_code_timestamp = None
    rx_proto = None
    rx_bitlength = None
    rx_pulselength = None
    
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
        attrs = config.attributes.fields
        board_name = attrs["board"].string_value
        self.board = dependencies[Board.get_resource_name(board_name)]
        self.data_pin = attrs["data_pin"].string_value
        self.gpio = self.board.gpio_pin_by_name(self.data_pin)
        return super().reconfigure(config, dependencies)
    
    async def receive_code(self):
        data_interrupt = self.board.DigitalInterrupt(name=self.data_pin)
        rising_edge_ts = -1
        falling_edge_ts = -1
        last_pulse_start_ts = -1
        last_pulse_end_ts = -1
        code = 0
        async for tick in await self.board.stream_ticks([data_interrupt]):
            # self.logger.info(f"Tick: {tick.time / 1000} microseconds {tick.high}")
            if tick.high:
                rising_edge_ts = tick.time
            else:
                falling_edge_ts = tick.time
                
            if falling_edge_ts > rising_edge_ts:
                # pulse has ended
                high_duration = falling_edge_ts - rising_edge_ts
                if high_duration < 2500000:
                    # filter out very short pulses as noise
                    high_duration = -1
                    rising_edge_ts = -1
                    falling_edge_ts = -1
                    continue
                
                # self.logger.info(f"Wave width: {high_duration / 1000} microseconds")
                
            
                distance_from_last_pulse_end = rising_edge_ts - last_pulse_end_ts

                if last_pulse_end_ts != -1:

                    last_pulse_duration = last_pulse_end_ts - last_pulse_start_ts

                    self.logger.info(f"Last pulse duration: {last_pulse_duration / 1000} us, distance from last pulse {distance_from_last_pulse_end / 1000} us")
                    if distance_from_last_pulse_end > 15000000:
                        # sync signal, set the reading
                        self.rx_code = code
                        self.logger.info(f"Sync signal, final code: {format(code, f'#024b')}")
                        code = 0
                    else:
                        # data signal

                        # short high followed by long low is a 0
                        if last_pulse_duration < 6000000 and distance_from_last_pulse_end > 6000000:
                            code <<= 1
                            self.logger.info(f"Signal 0, Binary code: {format(code, f'#024b')}")

                        # long high followed by short low is a 1
                        if last_pulse_duration > 6000000 and distance_from_last_pulse_end < 6000000:
                            code <<= 1
                            code |= 1
                            self.logger.info(f"Signal 1, Binary code: {format(code, f'#024b')}")
                
                last_pulse_start_ts = rising_edge_ts
                last_pulse_end_ts = falling_edge_ts

            
                
            
    async def get_readings(
        self,
        *,
        timeout: Optional[float] = None,
        **kwargs
    ) -> Mapping[str, SensorReading]:
        return {"code": str(self.rx_code)}

    async def do_command(
        self,
        command: Mapping[str, ValueTypes],
        *,
        timeout: Optional[float] = None,
        **kwargs
    ) -> Mapping[str, ValueTypes]:
        
        if command.get("receive"):
            await self.receive_code()
            return {"status": "received"}
        return {}

    async def get_geometries(
        self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None
    ) -> List[Geometry]:
        self.logger.error("`get_geometries` is not implemented")
        raise NotImplementedError()

