import asyncio
from viam.module.module import Module
try:
    from models.transmitter import Transmitter
except ModuleNotFoundError:
    # when running as local module with run.sh
    from .models.transmitter import Transmitter


if __name__ == '__main__':
    asyncio.run(Module.run_from_registry())
