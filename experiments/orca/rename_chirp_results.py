"""Shorten one existing v2 run's CSV names and update its metadata.

Preview by default; --apply renames files without changing their contents.
"""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import tempfile


def rename_results(run_dir, apply=False):
    run_dir = Path(run_dir)
    metadata_path = run_dir / "metadata.json"
    original = metadata_path.read_bytes()
    metadata = json.loads(original)
    joints, filenames = metadata["joints"], metadata["csv_files"]
    if (len(joints) != len(filenames) or len(set(joints)) != len(joints)
            or len(set(filenames)) != len(filenames)):
        raise ValueError("Require one unique CSV per joint; incomplete runs cannot be migrated")
    moves = []
    for joint, filename in zip(joints, filenames):
        if not joint or any(char not in "abcdefghijklmnopqrstuvwxyz0123456789_" for char in joint):
            raise ValueError(f"Invalid joint name: {joint!r}")
        target = f"{joint}.csv"
        if (Path(filename).name != filename
                or not (filename == target or (filename.startswith(f"chirp_{joint}_")
                                               and filename.endswith(".csv")))):
            raise ValueError(f"Filename does not match joint {joint}: {filename}")
        source_path, target_path = run_dir / filename, run_dir / target
        if not source_path.is_file() or source_path.is_symlink():
            raise ValueError(f"Missing or symlinked source: {source_path}")
        if filename != target:
            if target_path.exists() or target_path.is_symlink():
                raise FileExistsError(f"Will not overwrite {target_path}")
            moves.append((source_path, target_path))
    for source, target in moves:
        print(f"{source.name} -> {target.name}")
    if not apply or not moves:
        return moves

    metadata["csv_files"] = [f"{joint}.csv" for joint in joints]
    metadata.setdefault("run_id", run_dir.name)
    # Retain the UTC timestamp when filenames no longer carry it (e.g. in Gist).
    try:
        started_at = datetime.strptime(run_dir.name, "%Y%m%d_%H%M%S_%f").replace(tzinfo=timezone.utc)
    except ValueError:
        pass
    else:
        metadata.setdefault("started_at_utc", started_at.isoformat())
    backup = run_dir / ".metadata.before-rename.json"
    if backup.exists():
        if backup.read_bytes() != original:
            raise FileExistsError(f"Existing metadata backup differs: {backup}")
    else:
        with backup.open("xb") as stream:
            stream.write(original)

    completed = []
    with tempfile.NamedTemporaryFile(mode="w", dir=run_dir, prefix=".metadata-", delete=False) as stream:
        staged = Path(stream.name)
        json.dump(metadata, stream, indent=2)
        stream.write("\n")
    try:
        for source, target in moves:
            source.rename(target)
            completed.append((source, target))
        staged.replace(metadata_path)
    except BaseException:
        for source, target in reversed(completed):
            target.rename(source)
        raise
    finally:
        staged.unlink(missing_ok=True)
    return moves


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--apply", action="store_true")
    args = parser.parse_args()
    rename_results(args.run_dir, apply=args.apply)
    print("Migration complete." if args.apply else "Preview only. Add --apply to rename.")
