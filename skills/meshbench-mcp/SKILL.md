---
name: meshbench-mcp
description: Control a running MeshBench GUI over its local unauthenticated MCP server (HTTP + Boost.Beast). Use for inspecting the object tree, running File/Operations/View menu actions, and fetching PNG images of the current 3D scene.
---

# MeshBench MCP Skill

MeshBench is a Qt/VTK GUI for surface-mesh operations. While it is running it
starts a local MCP (Model Context Protocol) server so an LLM or other client
can inspect and drive the GUI.

- **MCP endpoint:** `POST http://127.0.0.1:8765/mcp`
- **Transport:** JSON-RPC 2.0 over HTTP (MCP Streamable HTTP style). Stateless;
  use a new HTTP request per message. `GET` on `/mcp` is not supported.
- **Authentication:** none, by design. The server binds to loopback.
- **Port:** `8765` by default. Override with `MESHBENCH_MCP_PORT` (see below).
- All commands execute synchronously on the GUI thread before the HTTP
  response is returned. Long operations can take many seconds.

## Port configuration

| `MESHBENCH_MCP_PORT` | Behaviour |
| --- | --- |
| unset | listen on `127.0.0.1:8765` |
| `0` | bind an ephemeral port (the startup log reports it) |
| `off`, `none`, `-1` | disable the MCP server |
| other integer | listen on that port |

`MESHBENCH_MCP_HOST` changes the bind address (default `127.0.0.1`). Only
change it if you understand the security implications: there is no auth.

The MeshBench **Log** panel prints the actual URL, for example:

```
MCP server listening on http://127.0.0.1:8765/mcp (unauthenticated, loopback only).
```

## MCP quick start

### 1. Initialize

```bash
curl -s http://127.0.0.1:8765/mcp \
  -H 'Content-Type: application/json' \
  -d '{"jsonrpc":"2.0","id":1,"method":"initialize",
       "params":{"protocolVersion":"2025-03-26",
                 "capabilities":{},"clientInfo":{"name":"my-llm","version":"1"}}}'
```

The response advertises `serverInfo.name = "meshbench"` and the `tools`
capability. Send `notifications/initialized` (no `id`) next if your client
expects it; notifications receive HTTP 202 and no body.

### 2. List tools

```bash
curl -s http://127.0.0.1:8765/mcp \
  -H 'Content-Type: application/json' \
  -d '{"jsonrpc":"2.0","id":2,"method":"tools/list"}'
```

### 3. Call a tool

```bash
curl -s http://127.0.0.1:8765/mcp \
  -H 'Content-Type: application/json' \
  -d '{"jsonrpc":"2.0","id":3,"method":"tools/call",
       "params":{"name":"scene_get_object_tree","arguments":{}}}'
```

A successful `tools/call` result looks like:

```json
{
  "content": [{"type": "text", "text": "Object tree: 3 stage(s), active stage 2."}],
  "structuredContent": { "...": "tool-specific JSON" },
  "isError": false
}
```

Tool failures set `isError: true` and put the reason in the text block.
Protocol errors use a normal JSON-RPC `error` object instead.

Image tools return an additional content block:

```json
{"type": "image", "mimeType": "image/png", "data": "<base64 PNG>"}
```

### 4. Minimal Python client

```python
import json, urllib.request

def rpc(method, params=None, id=1):
    payload = json.dumps({"jsonrpc": "2.0", "id": id,
                          "method": method, "params": params or {}}).encode()
    req = urllib.request.Request("http://127.0.0.1:8765/mcp", data=payload,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req) as r:
        return json.load(r)

def call(name, arguments=None):
    result = rpc("tools/call", {"name": name,
                                "arguments": arguments or {}})["result"]
    if result.get("isError"):
        raise RuntimeError(result["content"][0]["text"])
    # Prefer structured data; images are in additional content blocks.
    return result.get("structuredContent", result["content"][0]["text"])
```

## Convenience REST endpoints

These are for simple scripts and debugging. They expose the same command
layer, not a second implementation.

| Method + path | Purpose |
| --- | --- |
| `GET /api/tree` | object-tree JSON (same as `scene_get_object_tree`) |
| `GET /api/commands` | tool catalog: names, descriptions, input schemas |
| `GET /api/scene.png` | PNG of the current viewport (`?width=1600&height=1000`) |
| `GET /health` or `GET /` | server status and endpoint list |
| `POST /api/tool` | `{"name":"...","arguments":{...}}`; returns `{ok,message,error,data}`; images as `image_base64` |
| `POST /api/commands/<tool>` | body is the arguments object; one dedicated URL per menu action |

Example: run a tool directly.

```bash
curl -s -X POST http://127.0.0.1:8765/api/commands/operations_center \
  -H 'Content-Type: application/json' -d '{}'
```

Example: download a screenshot.

```bash
curl -s 'http://127.0.0.1:8765/api/scene.png?width=1600&height=1000' \
  -o /tmp/meshbench.png
```

## Object tree endpoint (most important)

`scene_get_object_tree` returns the pipeline ("object tree") state. Call it
after every operation instead of guessing indices. Shape:

```json
{
  "format_version": 1,
  "has_mesh": true,
  "stage_count": 3,
  "active_stage": 2,
  "visible_stages": [0, 2],
  "combine_selection": [],
  "stages": [
    {
      "index": 0,
      "label": "part.stl",
      "details": "",
      "timestamp": "2026-11-22T10:00:00",
      "parent": -1,
      "active": false,
      "visible": true,
      "combine_selected": false,
      "mesh": {
        "valid": true,
        "vertices": 1240,
        "faces": 2476,
        "edges": 3714,
        "boundary_edges": 0,
        "closed": true,
        "area": 12.5,
        "volume": 3.1,
        "bbox_min": [-1.0, -0.5, -0.5],
        "bbox_max": [1.0, 0.5, 0.5]
      },
      "opacity": null,
      "feature_edges": {"count": 0, "points": 0},
      "leak": {
        "probed": false, "has_leak": false, "hole_loops": 0, "regions": 0,
        "leak_paths": 0, "walk_paths": 0, "grid_res": 0, "cell_size": 0,
        "has_stored_routes": false, "has_stored_grid": false, "warnings": []
      },
      "probes": [],
      "face_count": 2476,
      "vertex_count": 1240
    }
  ],
  "editor_probes": [
    {"x": 0, "y": 0, "z": 0, "inside": true},
    {"x": 2, "y": 2, "z": 2, "inside": false}
  ],
  "selection": {"stage": -1, "count": 0, "faces": []}
}
```

Notes:

- Stage indices are how every command refers to pipeline stages.
- A stage's `parent` is the stage it was produced from (`-1` for independent
  sources such as imports and primitives).
- `active_stage` is what Operations act on.
- `visible_stages` is the eye-checkbox state; the active stage's overlays
  (probes, hole loops, leak paths) only render if it is also visible.
- `opacity` is `null` for automatic opacity, otherwise the pinned value.
- `editor_probes` is the current probe editor; `probes` on a stage is the set
  stored with that stage.
- `selection.faces` uses CGAL face indices of `selection.stage`.

## Tool reference

All tools are in the `tools/list` catalog. Schemas are authoritative; the
summaries below are for planning.

### File menu

| Tool | Arguments | GUI equivalent |
| --- | --- | --- |
| `file_open_mesh` | `path` | File > Open mesh (replaces the pipeline) |
| `file_import_meshes` | `paths: string[]` | File > Import meshes (append sources) |
| `file_import_feature_edges` | `path` | File > Import feature edges (eMesh) |
| `file_save_active_stage` | `path` | File > Save active stage (STL/OFF/OBJ by suffix) |
| `file_export_feature_edges` | `path` | File > Export feature edges as eMesh |
| `file_export_scene_image` | `path`, optional `width`, `height` | File > Export scene image |
| `file_clear` | none | File > Clear |
| `file_quit` | none | File > Quit (stops the server) |

Path rules: `path` is passed to the file APIs unchanged; relative paths
resolve against the MeshBench process working directory. Parent directories
must exist for exports. If `file_export_feature_edges` gets a path
without a suffix, `.eMesh` is appended.

### Operations menu

| Tool | Arguments (defaults) | GUI equivalent |
| --- | --- | --- |
| `operations_search` | none | Operations > Search operations (returns catalog) |
| `operations_boolean` | `path`, `operation`: `union`/`intersection`/`difference` | Boolean with another mesh |
| `operations_center` | none | Center at volume center |
| `operations_transform` | `translate: [x,y,z]` = 0s, `rotate_degrees: [x,y,z]` = 0s, `rotate_about_center` = true | Transform |
| `operations_alpha_wrap` | `alpha_denominator` = 500, `offset_denominator` = 1000 | Alpha wrap |
| `operations_isotropic_remesh` | `target_edge_length` (bbox diag / 50), `iterations` = 3, `feature_angle_degrees` = 30 | Isotropic remesh |
| `operations_extract_feature_edges` | `feature_angle_degrees` = 30, `include_boundary_edges` = true | Extract feature edges |
| `operations_snap_to_feature_edges` | `feature_edge_stage` (nearest earlier), `target_edge_length` (diag / 50), `min_facet_angle_degrees` = 20 | Snap to feature edges |
| `operations_autoclean` | `detail_size_ratio` = 0.35, `max_region_fraction` = 0.05, `min_small_faces` = 8, `grow_rings` = 1, `max_hole_edges` = 256 | Autoclean |
| `operations_slice` | `origin` (bbox center), `normal` = `[0,0,1]` | Slice with plane |
| `operations_combine` | `stages: int[]` (GUI selection if omitted), `mode`: `append`/`union` | Combine selected stages |
| `operations_detect_leaks` | `voxels_per_axis` = 48 | Detect leaks |
| `operations_patch_holes` | `max_loop_edges` = 64, `patch_all` = false | Patch holes |
| `operations_introduce_random_holes` | `triangles` = 10, `seed` = 42 | Introduce random holes |
| `operations_shrinkwrap` | `subdivisions` = 4, `iterations` = 25, `step` = 0.3, `smoothing` = 0.2, `even_spacing` = 0.0 | Shrinkwrap |
| `operations_create_primitive` | `shape`: `box`/`sphere`/`ellipsoid`/`cylinder`/`cone`/`torus`, `dimensions: number[]`, `mesh_size` = 0.1 | Create primitive shape |
| `operations_gmsh_remesh` | `target_element_size` = 0.1 | Remesh with Gmsh |

Primitive dimension order:

| Shape | Dimensions |
| --- | --- |
| `box` | `dx, dy, dz` |
| `sphere` | `r` |
| `ellipsoid` | `a, b, c` |
| `cylinder` | `r, h` |
| `cone` | `rBase, h` |
| `torus` | `R, r` |

Operation behaviour:

- Every operation appends a new stage and makes it active. Importing or
  creating a primitive appends a new independent source stage.
- Applying an operation while an older stage is active truncates the redo tail
  first (normal pipeline branching).
- Boolean operations need closed, triangulated meshes; they fail loudly instead
  of appending an empty stage.
- `operations_slice` keeps the negative side of the plane.
- `operations_detect_leaks` uses the probes in the probe editor. Run
  `scene_set_default_probes` first if none exist. It appends a `LeakProbe`
  stage carrying the mesh and the leak report, and turns the input stage's eye
  off (like ParaView).
- `operations_patch_holes` with `patch_all=false` re-probes and only fills
  loops that leak routes traverse; it needs probes. `patch_all=true` fills all
  boundary loops up to `max_loop_edges`.
- Some operations can legitimately return a no-op success (for example
  patching when no leak is found). Check the `message`/`structuredContent`.

### MCP-only low-level Gmsh CAD sessions

These tools are **not exposed in the MeshBench GUI**. They exist so an LLM
can perform CAD-like workflows (extruding, filleting, cutting holes, boolean
modeling, revolving) through the low-level Gmsh C++ OpenCASCADE API.

A session owns exactly **one pipeline stage**. All operations modify the
session's OCC geometry and then refresh that same stage in place, so the
object tree gains one node per session rather than one node per operation.

Lifecycle:

```
gmsh_session_begin            # creates one stage, starts the OCC session
gmsh_add_box / gmsh_extrude / gmsh_fillet / gmsh_boolean / ...
gmsh_list_entities            # discover tags for fillet/chamfer/edits
gmsh_session_end              # finalize; the stage keeps its final mesh
```

- Only one session may be active at a time.
- `gmsh_session_end` finalizes the session and leaves the stage in the tree.
- `gmsh_session_abort` finalizes the session and removes the stage by default
  (`remove_stage: false` keeps it).
- `gmsh_session_status` reports the active stage, mesh size and entity count.
- Operations return `structuredContent.stage` and `created` entity tags.
- The stage is re-meshed after every operation. If re-meshing fails, the
  geometry change is still applied and the result carries
  `structuredContent.mesh_error`; adjust `mesh_size` and call `gmsh_refresh`.
- The stage label starts with `GmshCAD` and its `details` show entity count
  and mesh size.
- If the stage is deleted, cleared, replaced by a new source, or truncated by
  another pipeline action, the session is automatically aborted.

Entities are `(dim, tag)` pairs. `gmsh_list_entities` returns `dim`, `tag`,
`type` and `bbox`. Dimensions: 0 = point, 1 = curve/edge, 2 = surface,
3 = volume. Primitive tools return their volume tag in `created`; use
`gmsh_list_entities {"dim":1}` for edge tags before filleting/chamfering.

#### Session control tools

| Tool | Arguments | Purpose |
| --- | --- | --- |
| `gmsh_session_begin` | `label`, `import_path`, `mesh_size` = 0.2 | Create the single stage and start OCC |
| `gmsh_session_end` | none | Finalize; keep the stage |
| `gmsh_session_abort` | `remove_stage` = true | Finalize; optionally remove stage |
| `gmsh_session_status` | none | Active?, stage, mesh size, entity count |
| `gmsh_set_mesh_size` | `mesh_size` | Change element size and refresh |
| `gmsh_refresh` | none | Re-mesh the current CAD geometry |
| `gmsh_list_entities` | `dim` = -1 | List CAD entities and tags |
| `gmsh_import_cad` | `path`, `highest_dim_only` = true | Import STEP/IGES/BREP |
| `gmsh_export_cad` | `path` | Export CAD (`.step`, `.brep`, ...) |

#### Low-level geometry tools

| Tool | Arguments | Purpose |
| --- | --- | --- |
| `gmsh_add_box` | `origin`, `size`, `tag` | OCC box volume |
| `gmsh_add_sphere` | `center`, `radius`, `tag` | OCC sphere volume |
| `gmsh_add_ellipsoid` | `center`, `radii`, `tag` | Scaled sphere |
| `gmsh_add_cylinder` | `base`, `axis`, `height`, `radius`, `angle_degrees`, `tag` | Cylinder |
| `gmsh_add_cone` | `base`, `axis`, `height`, `radius1`, `radius2`, `angle_degrees`, `tag` | Cone/frustum |
| `gmsh_add_torus` | `center`, `major_radius`, `minor_radius`, `angle_degrees`, `tag` | Torus |
| `gmsh_add_rectangle` | `origin`, `size: [dx,dy]`, `tag` | Planar surface in XY |
| `gmsh_add_disk` | `center`, `radius_x`, `radius_y`, `z_axis`, `x_axis`, `tag` | Planar disk/ellipse |
| `gmsh_extrude` | `dim` = 2, `tags`, `translate` | Extrude surfaces/curves |
| `gmsh_revolve` | `dim` = 2, `tags`, `axis_point`, `axis_direction`, `angle_degrees` | Revolve surfaces/curves |
| `gmsh_fillet` | `volumes`, `curves`, `radius`, `remove_volume` = true | Round edges |
| `gmsh_chamfer` | `volumes`, `curves`, `surfaces`, `distance`, `remove_volume` = true | Bevel edges |
| `gmsh_boolean` | `operation`: `cut`/`fuse`/`intersect`, `objects`, `tools`, `remove_object` = true, `remove_tool` = true | Boolean volumes |
| `gmsh_translate` | `dim` = 3, `tags`, `translate` | Translate entities |
| `gmsh_rotate` | `dim` = 3, `tags`, `axis_point`, `axis_direction`, `angle_degrees` | Rotate entities |
| `gmsh_scale` | `dim` = 3, `tags`, `center`, `factors` | Scale entities |
| `gmsh_mirror` | `dim` = 3, `tags`, `plane: [a,b,c,d]` | Mirror across a plane |
| `gmsh_copy` | `dim` = 3, `tags` | Copy entities, returns new tags |
| `gmsh_delete` | `dim` = 3, `tags`, `recursive` = true | Delete entities |

Angles are degrees. `tag` = -1 picks a new tag. Primitive `axis` values are
directions (normalized by MeshBench); `height` sets the length.

#### Example: plate with a through hole and a filleted edge

```
gmsh_session_begin {"mesh_size":0.5}
# -> created volume tag 1
gmsh_add_box {"origin":[-2,-1,-0.25],"size":[4,2,0.5]}
# -> created volume tag 2
gmsh_add_cylinder {"base":[0,0,-1],"axis":[0,0,1],"height":2,"radius":0.4}
# -> cut the cylinder out of the plate
gmsh_boolean {"operation":"cut","objects":[1],"tools":[2]}
# -> list edges, then fillet one of them
gmsh_list_entities {"dim":1}
gmsh_fillet {"volumes":[1],"curves":[3],"radius":0.1}
gmsh_session_end
```

#### Example: sketch-to-solid extrusion

```
gmsh_session_begin {"mesh_size":0.4}
gmsh_add_rectangle {"origin":[0,0,0],"size":[3,2]}
# rectangle is a surface tag, normally 1
gmsh_extrude {"dim":2,"tags":[1],"translate":[0,0,1.5]}
gmsh_session_end
```

#### Example: revolved profile

```
gmsh_session_begin {"mesh_size":0.4}
# A disk offset from the revolution axis creates a ring-like solid.
gmsh_add_disk {"center":[2,0,0],"radius_x":0.5}
gmsh_revolve {"dim":2,"tags":[1],"axis_point":[0,0,0],
              "axis_direction":[0,0,1],"angle_degrees":360}
gmsh_session_end
```

#### CAD modeling notes

- Start with `gmsh_session_begin`, then create or import geometry. An empty
  session stage has 0 faces until the first surface/volume exists.
- Use `gmsh_list_entities` after every boolean or fillet: OCC renumbers tags
  when `remove_object`/`remove_tool`/`remove_volume` delete inputs.
- `gmsh_boolean` operates on volume tags (dim 3). Cut a cylinder for holes;
  fuse for unions; intersect for common material.
- `gmsh_add_rectangle` and `gmsh_add_disk` create surfaces; extrude or revolve
  them into solids before boolean modeling.
- `gmsh_export_cad` is the way to preserve the parametric CAD model; the
  pipeline stage contains only the tessellated surface mesh.
- Prefer a coarser `mesh_size` while modeling; lower it before
  `gmsh_session_end` for the final stage if needed.

### View menu

| Tool | GUI equivalent |
| --- | --- |
| `view_from_pos_x` | View > View from +X |
| `view_from_neg_x` | View > View from -X |
| `view_from_pos_y` | View > View from +Y |
| `view_from_neg_y` | View > View from -Y |
| `view_from_pos_z` | View > View from +Z |
| `view_from_neg_z` | View > View from -Z |
| `view_reset_camera` | View > Reset camera |

The six standard views keep the current focal point and camera distance and
snap the camera direction (Z-up conventions as in the GUI).

### Scene control and inspection

| Tool | Arguments | Purpose |
| --- | --- | --- |
| `scene_get_object_tree` | none | Full object-tree JSON |
| `scene_get_image` | optional `width`, `height` | PNG image content block of the viewport |
| `scene_set_active_stage` | `index` | Choose the stage Operations act on |
| `scene_set_stage_visible` | `index`, `visible` | Eye checkbox |
| `scene_set_stage_opacity` | `index`, `opacity` (0-1, `-1` = auto) | Pin stage opacity |
| `scene_set_combine_selection` | `stages: int[]` | Replace Combine checkboxes |
| `scene_set_probes` | `probes: [{x,y,z,inside}]` | Replace leak probes |
| `scene_set_default_probes` | none | bbox center (inside) + beyond bbox (outside) |
| `scene_get_log` | `limit` = 200 | Tail of the GUI log |

`width`/`height` must both be `0`/omitted (current widget size) or both be
positive. Image export temporarily resizes the render window and restores it.

## Recommended LLM workflows

### Inspect before acting

1. `scene_get_object_tree` — learn `stage_count`, `active_stage`, labels and
   mesh stats.
2. `scene_get_image` — see the current viewport.
3. If the scene is empty, load or create geometry:
   `file_open_mesh`, `file_import_meshes`, or `operations_create_primitive`.

### Run an operation and verify

```
tools/call operations_transform {"translate":[0,0,1]}
tools/call scene_get_object_tree {}
tools/call scene_get_image {}
```

The new stage is appended and becomes active. If the result is wrong, use
`scene_set_active_stage` to go back and branch from an earlier stage, or
`file_clear` to start over. There is no undo beyond pipeline branching.

### Leak detection and repair

```
scene_set_default_probes
operations_detect_leaks {"voxels_per_axis":48}
scene_get_object_tree            # look at the LeakProbe stage's leak object
operations_patch_holes {"max_loop_edges":64}
operations_detect_leaks          # verify the patched stage is sealed
```

If no inside/outside probe is desired, pass `patch_all: true` to patch every
boundary loop instead (fixed holes only, not path-driven).

### Feature edges and snapping

```
operations_extract_feature_edges {"feature_angle_degrees":30}
# note the new stage index from scene_get_object_tree
operations_alpha_wrap
operations_snap_to_feature_edges {"feature_edge_stage":<edge stage>,
                                  "target_edge_length":0.2,
                                  "min_facet_angle_degrees":20}
```

`operations_snap_to_feature_edges` copies the feature-edge set before appending
its result, so branching from an older active stage cannot invalidate the
source set.

### Screenshot for the user

```
scene_get_image {"width":1600,"height":1000}
```

or save to disk with `file_export_scene_image {"path":"/tmp/scene.png"}`.

## Error handling and safety

- Tool errors return `isError: true` with a text reason; do not parse
  `structuredContent` in that case.
- Unknown tool names, malformed arguments and out-of-range stage indices fail
  without modifying the document.
- Commands are serialized on the GUI thread. A second concurrent request waits
  for the first; commands are not atomic as a group, so re-read the tree after
  multi-step changes.
- The server is local and unauthenticated. Do not expose the port beyond
  loopback.
- `file_quit` closes the window and stops the server; the response is sent
  first, but subsequent requests will fail.
- Output paths must be writable by the MeshBench process. Exports do not create
  missing parent directories.
- `file_export_feature_edges` requires the active stage to carry feature edges
  (run or import them first).

## Minimal MCP client configuration

Most MCP clients accept a URL-based HTTP server entry:

```json
{
  "mcpServers": {
    "meshbench": {
      "url": "http://127.0.0.1:8765/mcp"
    }
  }
}
```

If your client only supports stdio servers, wrap the HTTP endpoint with a
stdio-to-HTTP MCP bridge, or use `curl` from a shell tool.
