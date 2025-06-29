from collections import namedtuple
import time
from typing import (Any, ClassVar, Dict, Final, List, Mapping, Optional,
                    Sequence)

from typing_extensions import Self
from viam.components.generic import *
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import Geometry, ResourceName
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.components.board import Board
from viam.utils import ValueTypes
from viam.logging import getLogger
import asyncio

MAX_CHANGES = 67

Protocol = namedtuple('Protocol',
                      ['pulselength', # microseconds
                       'sync_high', 'sync_low',
                       'zero_high', 'zero_low',
                       'one_high', 'one_low'])
PROTOCOLS = (None,
             Protocol(3500, 1, 31, 1, 3, 3, 1),
             Protocol(650, 1, 10, 1, 2, 2, 1),
             Protocol(100, 30, 71, 4, 11, 9, 6),
             Protocol(380, 1, 6, 1, 3, 3, 1),
             Protocol(500, 6, 14, 1, 2, 2, 1),
             Protocol(200, 1, 10, 1, 5, 1, 1))


class Transmitter(Generic, EasyResource):
    # To enable debug-level logging, either run viam-server with the --debug option,
    # or configure your resource/machine to display debug logs.
    MODEL: ClassVar[Model] = Model(ModelFamily("grant-dev", "433mhz-rf"), "transmitter")

    board: Board
    data_pin: str

    tx_protocol: int = 1
    # microseconds
    tx_pulselength: int = PROTOCOLS[tx_protocol].pulselength
    tx_repeat: int = 10
    tx_length: int = 24
    

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
        return super().reconfigure(config, dependencies)
    
    async def transmit_code(self, code: int, gpio: Board.GPIOPin):
        rawcode = format(code, f'#0{self.tx_length + 2}b')[2:]
        self.logger.info(f"Transmitting code: {str(code)}")
        await self.transmit_binary(rawcode, gpio)
        self.logger.info(f"Transmission complete")
        return True
    
    async def transmit_binary(self, rawcode: str, gpio: Board.GPIOPin):
        self.logger.info(f"Transmitting binary: {str(rawcode)}")
        for _ in range(0, self.tx_repeat):
            for byte in range(0, self.tx_length):
                if rawcode[byte] == '0':
                    await self.transmit_0(gpio)
                else:
                    await self.transmit_1(gpio)
            await self.transmit_sync(gpio)
        return True
    
    async def transmit_0(self, gpio: Board.GPIOPin):
        return await self.transmit_waveform(
            PROTOCOLS[self.tx_protocol].zero_high, 
            PROTOCOLS[self.tx_protocol].zero_low, 
            gpio)

    async def transmit_1(self, gpio: Board.GPIOPin):
        return await self.transmit_waveform(
            PROTOCOLS[self.tx_protocol].one_high, 
            PROTOCOLS[self.tx_protocol].one_low, 
            gpio)
    
    async def transmit_sync(self, gpio: Board.GPIOPin):
        return await self.transmit_waveform(
            PROTOCOLS[self.tx_protocol].sync_high, 
            PROTOCOLS[self.tx_protocol].sync_low, 
            gpio)

    async def transmit_waveform(self, highpulses: int, lowpulses: int, gpio: Board.GPIOPin):
        await gpio.set(True)
        await self._sleep((highpulses * self.tx_pulselength) / 1000000)
        await gpio.set(False)
        await self._sleep((lowpulses * self.tx_pulselength) / 1000000)
        return True
    
    async def _sleep(self, delay):
        _delay = delay / 100
        end = time.time() + delay - _delay
        while time.time() < end:
            await asyncio.sleep(_delay) # seconds


    async def do_command(
        self,
        command: Mapping[str, ValueTypes],
        *,
        timeout: Optional[float] = None,
        **kwargs
    ) -> Mapping[str, ValueTypes]:
        gpio = await self.board.gpio_pin_by_name(self.data_pin)
        code = command.get("code")
        if code:
            await self.transmit_code(int(code), gpio)
        else:
            return {"success": False, "error": "code is required"}
        return {"success": True}

    async def get_geometries(
        self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None
    ) -> List[Geometry]:
        self.logger.error("`get_geometries` is not implemented")
        raise NotImplementedError()

