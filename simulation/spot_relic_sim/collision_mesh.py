"""Prepare face-referenced OBJ geometry for MuJoCo's vertex-based convex hulls."""
from pathlib import Path

from .contract import sha256


def compact_obj(source, destination):
    """Remove unused positions and remap face indices, preserving the surfaces.

    MuJoCo builds its hull from all positions, including orphan vertices left by
    mesh editing. Isaac's import retains face-referenced positions instead.
    Normals/UVs remain unchanged; only the position component of each face index
    is remapped. Negative position indices are resolved at their source line.
    """
    source, destination = Path(source), Path(destination)
    lines = source.read_text().splitlines(keepends=True)
    vertices, used, faces = 0, set(), {}
    for line_number, line in enumerate(lines):
        terms = line.partition("#")[0].split()
        if not terms:
            continue
        if terms[0] == "v":
            vertices += 1
        elif terms[0] == "f":
            if len(terms) < 4:
                raise ValueError(f"OBJ face needs at least three positions: {source}")
            face = []
            for token in terms[1:]:
                position, slash, rest = token.partition("/")
                index = int(position)
                if index == 0:
                    raise ValueError(f"OBJ position indices cannot be zero: {source}")
                index = index - 1 if index > 0 else vertices + index
                used.add(index)
                face.append((index, slash + rest))
            faces[line_number] = face
        elif terms[0] in ("l", "p", "curv", "surf"):
            raise ValueError(f"Collision OBJ must contain face geometry only: {source}")
    if not used or min(used) < 0 or max(used) >= vertices:
        raise ValueError(f"Collision OBJ has missing or invalid face positions: {source}")
    remap = {old: new for new, old in enumerate(sorted(used), start=1)}
    output, index = [], 0
    for line_number, line in enumerate(lines):
        terms = line.partition("#")[0].split()
        if terms and terms[0] == "v":
            if index in used:
                output.append(line)
            index += 1
        elif line_number in faces:
            output.append("f " + " ".join(f"{remap[i]}{suffix}" for i, suffix in faces[line_number]) + "\n")
        else:
            output.append(line)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text("".join(output))
    return dict(source_sha256=sha256(source), generated_sha256=sha256(destination),
                original_vertices=vertices, referenced_vertices=len(used), removed_vertices=vertices - len(used))
