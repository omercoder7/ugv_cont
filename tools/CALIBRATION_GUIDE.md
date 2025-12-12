# Collision Calibration Tool - Quick Reference

## Commands

| Command | Description |
|---------|-------------|
| `./calibrate_collision.py` | Interactive mode (menu-driven) |
| `./calibrate_collision.py --list` | List all calibrated obstacles with stats |
| `./calibrate_collision.py --show` | Show current thresholds and calibration |
| `./calibrate_collision.py --clear 30` | Collect 30s of clear path baseline |
| `./calibrate_collision.py --obstacle NAME 15` | Calibrate obstacle NAME for 15s (hard by default) |
| `./calibrate_collision.py --obstacle NAME 15 --soft` | Calibrate as soft obstacle |
| `./calibrate_collision.py --obstacle NAME 15 --hard` | Calibrate as hard obstacle |
| `./calibrate_collision.py --delete NAME` | Delete obstacle NAME (re-calibrate fresh) |
| `./calibrate_collision.py --compute` | Recompute thresholds from all data |
| `./calibrate_collision.py --reset` | Reset ALL calibration data |

## Controls During Collection

| Key | Action |
|-----|--------|
| `SPACE` | Pause/resume collection |
| `q` | Stop and save |
| `Ctrl+C` | Cancel (stops robot) |

## Obstacle Categories

| Category | Description | Example |
|----------|-------------|---------|
| **clear** | Robot moves freely | Hallway, open room |
| **soft** | Partial resistance, robot still moves slowly | Carpet edge, thick rug, pillow |
| **hard** | Fully blocks robot | Wall, glass door, furniture leg |

## Workflow

1. **Collect baseline**: `./calibrate_collision.py --clear 30`
   - Drive robot in open area for 30 seconds

2. **Calibrate obstacles**: `./calibrate_collision.py --obstacle glass_door 15 --hard`
   - Drive robot into obstacle, let it get blocked

3. **Add more samples** (same obstacle): Just run the same command again - samples are appended!

4. **Re-calibrate fresh**:
   ```bash
   ./calibrate_collision.py --delete glass_door
   ./calibrate_collision.py --obstacle glass_door 15 --hard
   ```

## How It Works

The CollisionVerifier tracks **movement efficiency**:
- `efficiency = actual_distance_moved / expected_distance`
- Clear path: efficiency ~100%
- Blocked: efficiency drops to ~2-10%

Thresholds trigger detection:
- **Suspicious** (< 60%): Robot slows down and probes
- **Blocked** (< 40%): Robot confirms obstacle, backs up, turns
