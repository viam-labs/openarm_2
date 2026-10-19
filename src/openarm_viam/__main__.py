import asyncio

from viam.module.module import Module

import openarm_viam.models  # noqa: F401


def main() -> None:
    asyncio.run(Module.run_from_registry())


if __name__ == "__main__":
    main()
