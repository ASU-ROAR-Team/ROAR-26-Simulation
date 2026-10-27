import os
import signal
import psutil

for p in psutil.process_iter(['pid', 'cmdline']):
    cmd = " ".join(p.info['cmdline'] or [])
    if 'add_world.py' in cmd or 'dynamic_rock_dropper.py' in cmd or 'generate_worlds.py' in cmd:
        # Don't kill our own cleanup script if it matches somehow
        if 'cleanup_zombies.py' not in cmd:
            try:
                os.kill(p.info['pid'], signal.SIGKILL)
                print(f"Killed {p.info['pid']}: {cmd}")
            except Exception as e:
                pass

# Also kill ignition gazebo
for p in psutil.process_iter(['pid', 'name', 'cmdline']):
    cmd = " ".join(p.info['cmdline'] or [])
    if 'ign gazebo' in cmd or 'ruby' in cmd:
        try:
            os.kill(p.info['pid'], signal.SIGKILL)
        except Exception:
            pass
