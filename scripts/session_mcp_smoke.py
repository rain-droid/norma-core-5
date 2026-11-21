"""Read-only end-to-end MCP smoke test against a running session service."""
import asyncio
import json
import os
import sys
from pathlib import Path

from mcp import ClientSession
from mcp.client.stdio import StdioServerParameters, stdio_client


async def main():
    env = {**os.environ, 'SO101_CLEANUP_MODE': 'session'}
    server = StdioServerParameters(command=sys.executable, args=['-m', 'so101_station.cleanup.mcp_server'], env=env)
    async with stdio_client(server) as (read, write):
        async with ClientSession(read, write) as session:
            await session.initialize()
            listing = await session.list_tools()
            assert {t.name for t in listing.tools} == {'status', 'observe', 'act', 'assess', 'stop', 'set_torque', 'probe_alignment', 'align_target', 'prepare_alignment', 'select_target', 'locate_target', 'park'}
            result = await session.call_tool('observe', {})
            assert not result.is_error, result
            assert sum(c.type == 'image' for c in result.content) == 2
            text = json.loads(result.content[0].text)
            obs = text['result']
            print(json.dumps({'tools': [t.name for t in listing.tools], 'images': 2,
                              'run_id': obs['run_id'], 'stage': obs['stage'], 'views': obs['views']}, indent=2))


if __name__ == '__main__':
    asyncio.run(main())
