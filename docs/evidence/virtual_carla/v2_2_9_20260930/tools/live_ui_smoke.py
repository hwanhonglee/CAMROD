"""Real UI/CARLA smoke. No mocked sensors, teleportation, or direct drive API.

Commands are pointer/key events through the production administrator panel.
The CARLA client only reads the actual Ranger transform/velocity for evidence.
Always release keys, disarm and request normal UI STOP before exiting.
"""
import asyncio
import json
import math
import os
from pathlib import Path
import time

import carla
from playwright.async_api import async_playwright

OUT = Path(os.environ['SMOKE_OUTPUT'])


async def main():
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / 'frames').mkdir(exist_ok=True)
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(5)
    world = client.get_world()
    world.wait_for_tick(5)
    actors = [a for a in world.get_actors()
              if a.type_id == 'vehicle.ranger.default'
              and a.attributes.get('role_name') == 'ego_vehicle']
    assert len(actors) == 1, f'Expected exactly one Ranger, found {len(actors)}'
    actor = actors[0]

    def pose():
        t, v = actor.get_transform(), actor.get_velocity()
        return dict(x=t.location.x, y=t.location.y, z=t.location.z,
                    yaw=t.rotation.yaw, speed_mps=math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z))

    report = dict(actor_id=actor.id, map=world.get_map().name, scenarios=[], samples=[])
    async with async_playwright() as p:
        browser = await p.chromium.launch(executable_path='/usr/bin/google-chrome',
            headless=True, args=['--no-sandbox'])
        page = await browser.new_page(viewport={'width': 1600, 'height': 1100}, locale='ko-KR')
        frame = 0

        async def observe(seconds, label):
            nonlocal frame
            until = time.monotonic() + seconds
            while time.monotonic() < until:
                report['samples'].append(dict(label=label, wall_time=time.time(), **pose()))
                await page.screenshot(path=str(OUT / 'frames' / f'{frame:04d}.png'))
                frame += 1
                await asyncio.sleep(0.3)

        try:
            await page.goto('http://127.0.0.1:8010', wait_until='networkidle')
            await page.screenshot(path=str(OUT / '01_live_robot_ui.png'))
            await page.locator('.diag-secret-zone-global').hover()
            await page.mouse.down()
            await page.wait_for_timeout(1800)
            await page.mouse.up()
            await page.get_by_placeholder('아이디를 입력하세요').fill('admin')
            await page.locator('#login-pw-input').fill('1234')
            await page.locator('.login-submit-btn').click()
            await page.locator('[data-ui="operator-diagnostic-tab-camera"]').click()
            await page.wait_for_timeout(1800)
            await page.screenshot(path=str(OUT / '02_real_carla_cameras.png'))
            await page.locator('[data-ui="manual-drive-toggle"]').click()
            await page.locator('[data-ui="manual-drive-arm"]').click()
            await page.wait_for_function("document.querySelector('[data-ui=manual-drive-panel]')?.dataset.armed === 'true'")
            for name, keys in [('straight', ['w']), ('reverse', ['s']),
                               ('crab_left', ['z']), ('crab_right', ['c']),
                               ('zero_left', ['a']), ('zero_right', ['d'])]:
                start = pose()
                for key in keys:
                    await page.keyboard.down(key)
                await observe(2.5, name)
                for key in keys:
                    await page.keyboard.up(key)
                await page.keyboard.press('Space')
                await page.wait_for_timeout(1000)
                end = pose()
                yaw = (end['yaw'] - start['yaw'] + 180) % 360 - 180
                report['scenarios'].append(dict(name=name, start=start, end=end,
                    distance_m=math.hypot(end['x']-start['x'], end['y']-start['y']),
                    yaw_delta_deg=yaw))
            await page.screenshot(path=str(OUT / '03_manual_motion_finished.png'))
            await page.keyboard.press('Escape')
            await page.get_by_role('button', name='스냅샷', exact=True).click()
            await page.wait_for_timeout(1000)
            await page.screenshot(path=str(OUT / '04_snapshot_buffer.png'), full_page=True)
            for endpoint, key in [('ui/state', 'ui_state'), ('api/admin/snapshot/status', 'snapshot')]:
                response = await page.request.get('http://127.0.0.1:8010/' + endpoint)
                report[key] = await response.json()
            report['completed'] = True
        except Exception as error:
            report['error'] = repr(error)
            await page.screenshot(path=str(OUT / 'failure.png'), full_page=True)
            print((await page.locator('body').inner_text())[-3500:])
            raise
        finally:
            for key in ('w', 's', 'a', 'd', 'z', 'c'):
                await page.keyboard.up(key)
            await page.keyboard.press('Space')
            await page.keyboard.press('Escape')
            await page.request.post('http://127.0.0.1:8010/ui/stop')
            report['final_pose'] = pose()
            (OUT / 'actual_runtime.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
            await browser.close()
    print(json.dumps(report['scenarios'], indent=2))


asyncio.run(main())
