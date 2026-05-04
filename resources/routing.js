//////////// PURE UTILITIES ////////////

function clamp(x, lo, hi) {
  return Math.max(lo, Math.min(hi, x));
}

function nextFrame() {
  return new Promise((resolve) => requestAnimationFrame(resolve));
}

function sameCoordLL(a, b) {
  if (!a || !b) return false;
  return a[0] === b[0] && a[1] === b[1];
}

/// Degrees-to-radians factor used by haversineM; hoisted to avoid per-call allocation.
const TO_RAD = Math.PI / 180;

//////////// CHOROPLETH COLOURING ////////////

/// Each bin: upper bound in minutes (exclusive) + RGBA fill color.
const CHORO_BINS = [
  { max: 4,        label: "0 – 4 min",   color: [0, 160, 100, 0.8] },
  { max: 8,        label: "4 – 8 min",   color: [120, 210, 60, 0.8] },
  { max: 12,       label: "8 – 12 min",  color: [255, 210, 40, 0.8] },
  { max: 16,       label: "12 – 16 min", color: [255, 120, 20, 0.8] },
  { max: Infinity, label: "16 + min",    color: [200, 30, 30, 0.8] },
];

/// Returns the RGBA color for a drive time given in minutes.
function choroplethColorDiscrete(minutes) {
  if (!Number.isFinite(minutes)) return [100, 100, 100, 0.5]; // unreachable
  for (const bin of CHORO_BINS) {
    if (minutes < bin.max) return bin.color;
  }
  return CHORO_BINS[CHORO_BINS.length - 1].color;
}

/// Build a static discrete-bin legend DOM element.
function buildChoroplethLegend() {
  const wrap = document.createElement("div");
  Object.assign(wrap.style, {
    background: "rgba(255,255,255,0.92)",
    backdropFilter: "blur(2px)",
    border: "1px solid rgba(0,0,0,0.15)",
    borderRadius: "10px",
    padding: "10px 12px",
    boxShadow: "0 1px 4px rgba(0,0,0,0.18)",
    fontFamily: "Arial, sans-serif",
    fontSize: "12px",
    lineHeight: "1.4",
    minWidth: "160px",
  });

  const title = document.createElement("div");
  title.textContent = "Drive time to nearest destination";
  Object.assign(title.style, { fontWeight: "700", marginBottom: "8px" });
  wrap.appendChild(title);

  for (const bin of CHORO_BINS) {
    const row = document.createElement("div");
    Object.assign(row.style, {
      display: "flex",
      alignItems: "center",
      gap: "8px",
      marginBottom: "4px",
    });

    const swatch = document.createElement("div");
    const [r, g, b, a] = bin.color;
    Object.assign(swatch.style, {
      width: "18px",
      height: "12px",
      borderRadius: "3px",
      border: "1px solid rgba(0,0,0,0.18)",
      background: `rgba(${r},${g},${b},${a})`,
      flexShrink: "0",
    });

    const lbl = document.createElement("span");
    lbl.textContent = bin.label;

    row.appendChild(swatch);
    row.appendChild(lbl);
    wrap.appendChild(row);
  }

  return wrap;
}

//////////// DATA STRUCTURES ////////////

class MinHeap {
  constructor() {
    this.h = [];
  }
  get size() {
    return this.h.length;
  }

  push(value, priority) {
    const h = this.h;
    h.push([priority, value]);

    for (let i = h.length - 1; i > 0; ) {
      const parent = (i - 1) >> 1;
      if (h[parent][0] <= h[i][0]) break;
      [h[parent], h[i]] = [h[i], h[parent]];
      i = parent;
    }
  }

  pop() {
    const h = this.h;
    if (!h.length) return null;

    const top = h[0];
    const last = h.pop();

    if (h.length) {
      h[0] = last;

      for (let i = 0; ; ) {
        const left = i * 2 + 1;
        const right = left + 1;
        let best = i;

        if (left < h.length && h[left][0] < h[best][0]) best = left;
        if (right < h.length && h[right][0] < h[best][0]) best = right;

        if (best === i) break;
        [h[best], h[i]] = [h[i], h[best]];
        i = best;
      }
    }

    return top;
  }
}

//////////// PATHFINDING ////////////

function astar(graph, start, goal, closedEdges) {
  const n = graph.nodes_xy.length;

  const score = new Float64Array(n);
  score.fill(Infinity);
  score[start] = 0;

  const prevNode = new Int32Array(n);
  const prevEdge = new Int32Array(n);
  const prevRev = new Int32Array(n);
  prevNode.fill(-1);
  prevEdge.fill(-1);
  prevRev.fill(-1);

  const hScale = graph.heuristic_scale ?? 1.0;
  const [gx, gy] = graph.nodes_xy[goal];
  const h = (i) => {
    const [x, y] = graph.nodes_xy[i];
    return Math.hypot(x - gx, y - gy) * hScale;
  };

  const open = new MinHeap();
  open.push(start, h(start));

  while (open.size) {
    const [f, u] = open.pop();
    if (u === goal) break;

    const gu = score[u];
    if (f !== gu + h(u)) continue;

    for (const [v, w, edgeIdx, rev] of graph.adj[u]) {
      if (closedEdges.has(edgeIdx)) continue;

      const alt = gu + w;
      if (alt < score[v]) {
        score[v] = alt;
        prevNode[v] = u;
        prevEdge[v] = edgeIdx;
        prevRev[v] = rev;
        open.push(v, alt + h(v));
      }
    }
  }

  if (!Number.isFinite(score[goal])) return null;

  const path = [];
  /// Guard against corrupt prevNode chains; path length can never exceed node count.
  for (let v = goal; v !== start; v = prevNode[v]) {
    if (path.length > n)
      throw new Error("astar: cycle detected in prevNode back-trace");
    path.push({ edge_idx: prevEdge[v], rev: prevRev[v] });
  }
  path.reverse();

  return { path, dist_m: score[goal] };
}

function stitchEdges(graph, path) {
  const coords = [];

  for (const { edge_idx, rev } of path) {
    const seg = graph.edges_coords_ll[edge_idx];

    if (rev) {
      const skipFirst =
        coords.length > 0 &&
        coords[coords.length - 1][0] === seg[seg.length - 1][0] &&
        coords[coords.length - 1][1] === seg[seg.length - 1][1];
      for (let j = seg.length - 1 - (skipFirst ? 1 : 0); j >= 0; j--)
        coords.push(seg[j]);
      continue;
    }

    if (!coords.length) {
      Array.prototype.push.apply(coords, seg);
      continue;
    }

    const last = coords[coords.length - 1];
    const joined = last[0] === seg[0][0] && last[1] === seg[0][1];
    Array.prototype.push.apply(coords, joined ? seg.slice(1) : seg);
  }

  return coords;
}

//////////// REVERSE DIJKSTRA ////////////

/// Build adjacency list for reverse graph: reverseAdj[v] = [[u, w, edgeIdx], ...]
function buildReverseAdj(graph) {
  const n = graph.nodes_xy.length;
  const radj = Array.from({ length: n }, () => []);
  for (let u = 0; u < graph.adj.length; u++) {
    for (const [v, w, edgeIdx] of graph.adj[u] || []) {
      radj[v].push([u, w, edgeIdx]);
    }
  }
  return radj;
}

/// Multi-source Dijkstra on the reverse graph seeded from all destination nodes.
/// Returns {dist, nearestDest, nextHop, nextHopEdge} arrays indexed by node.
/// nextHop[v]     = the next node on v's shortest path to its nearest destination.
/// nextHopEdge[v] = the graph edgeIdx of that step (used for flow aggregation).
function reverseMultiSourceDijkstra(reverseAdj, destNodes) {
  const n = reverseAdj.length;
  const dist = new Float64Array(n).fill(Infinity);
  const nearestDest = new Int32Array(n).fill(-1);
  const nextHop = new Int32Array(n).fill(-1);
  const nextHopEdge = new Int32Array(n).fill(-1);
  const heap = new MinHeap();

  for (const { node, id } of destNodes) {
    if (dist[node] === Infinity) {
      dist[node] = 0;
      nearestDest[node] = id;
      heap.push(node, 0);
    } else if (dist[node] === 0) {
      // Two destinations share the same road node — let the later one claim it.
      nearestDest[node] = id;
    }
  }

  while (heap.size) {
    const [d, u] = heap.pop();
    if (d > dist[u]) continue;
    for (const [v, w, edgeIdx] of reverseAdj[u]) {
      const nd = dist[u] + w;
      if (nd < dist[v]) {
        dist[v] = nd;
        nearestDest[v] = nearestDest[u];
        nextHop[v] = u;
        nextHopEdge[v] = edgeIdx;
        heap.push(v, nd);
      }
    }
  }

  return { dist, nearestDest, nextHop, nextHopEdge };
}

/// Aggregate flow on each graph edge by propagating origin counts up the nextHop tree.
/// Returns a Map of edgeIdx → total flow count (origins routed through that edge).
/// O(N) in the number of graph nodes.
function buildEdgeFlow(nextHop, nextHopEdge, originNodes) {
  const n = nextHop.length;

  // Seed flow counts: 1 per origin node.
  const flow = new Float64Array(n);
  for (const node of originNodes) flow[node] = 1;

  // Count children in the nextHop tree so we can process leaves first.
  const pending = new Int32Array(n);
  for (let v = 0; v < n; v++) {
    const p = nextHop[v];
    if (p >= 0) pending[p]++;
  }

  // Kahn-style leaf-first propagation.
  const queue = [];
  for (let v = 0; v < n; v++) {
    if (pending[v] === 0 && nextHop[v] >= 0) queue.push(v);
  }
  while (queue.length) {
    const v = queue.pop();
    const p = nextHop[v];
    if (p < 0) continue;
    flow[p] += flow[v];
    if (--pending[p] === 0 && nextHop[p] >= 0) queue.push(p);
  }

  // Collect per-edge totals (sum both directions of the same physical segment).
  const edgeFlow = new Map();
  for (let v = 0; v < n; v++) {
    if (flow[v] === 0 || nextHop[v] < 0 || nextHopEdge[v] < 0) continue;
    const eid = nextHopEdge[v];
    edgeFlow.set(eid, (edgeFlow.get(eid) || 0) + flow[v]);
  }
  return edgeFlow;
}

/// Incremental Dijkstra from one new destination; prunes where it cannot improve.
function incrementalDijkstra(
  reverseAdj,
  newDestNode,
  newDestId,
  dist,
  nearestDest,
) {
  if (newDestNode < 0) return;
  const heap = new MinHeap();
  if (0 < dist[newDestNode]) {
    dist[newDestNode] = 0;
    nearestDest[newDestNode] = newDestId;
    heap.push(newDestNode, 0);
  }
  while (heap.size) {
    const [d, u] = heap.pop();
    if (d > dist[u]) continue;
    for (const [v, w] of reverseAdj[u]) {
      const nd = dist[u] + w;
      if (nd < dist[v]) {
        dist[v] = nd;
        nearestDest[v] = newDestId;
        heap.push(v, nd);
      }
    }
  }
}

//////////// GRAPH GEOMETRY ////////////

function haversineM(a, b) {
  const lon1 = a[0] * TO_RAD,
    lat1 = a[1] * TO_RAD;
  const lon2 = b[0] * TO_RAD,
    lat2 = b[1] * TO_RAD;

  const dlon = lon2 - lon1;
  const dlat = lat2 - lat1;

  const s =
    Math.sin(dlat / 2) ** 2 +
    Math.cos(lat1) * Math.cos(lat2) * Math.sin(dlon / 2) ** 2;

  return 2 * 6371000 * Math.asin(Math.sqrt(s));
}

// Recover lon/lat for each node using adj + stored edge geometries
function buildNodeCoordsLL(graph) {
  const n = graph.nodes_xy.length;
  const out = new Array(n).fill(null);

  for (let u = 0; u < graph.adj.length; u++) {
    for (const [v, , edgeIdx, rev] of graph.adj[u] || []) {
      const seg = graph.edges_coords_ll[edgeIdx];
      if (!seg || !seg.length) continue;

      const first = seg[0];
      const last = seg[seg.length - 1];

      if (!rev) {
        if (!out[u]) out[u] = first.slice();
        if (!out[v]) out[v] = last.slice();
      } else {
        if (!out[u]) out[u] = last.slice();
        if (!out[v]) out[v] = first.slice();
      }
    }
  }

  let missingCount = 0;
  let firstMissing = -1;
  for (let i = 0; i < out.length; i++) {
    if (!out[i]) {
      missingCount++;
      if (firstMissing < 0) firstMissing = i;
    }
  }
  if (missingCount) {
    throw new Error(
      `Could not recover lon/lat for ${missingCount} node(s); first missing node: ${firstMissing}`,
    );
  }

  return out;
}

/// Snap a projected-CRS point [cx, cy] to the nearest graph node using nodes_xy (same CRS).
/// Avoids any lon/lat conversion or projection module dependency.
function findNearestNodeXY(graph, cx, cy) {
  const nodes = graph.nodes_xy;
  let bestNode = -1;
  let bestDist = Infinity;
  for (let i = 0; i < nodes.length; i++) {
    const [x, y] = nodes[i];
    const d = Math.hypot(x - cx, y - cy);
    if (d < bestDist) {
      bestDist = d;
      bestNode = i;
    }
  }
  return bestNode;
}

// nodeCoordsLL is set by app.js at init time; safe to read here after that.
function findNearestNode(rawLL) {
  let bestNode = -1;
  let bestDist = Infinity;

  for (let i = 0; i < nodeCoordsLL.length; i++) {
    const ll = nodeCoordsLL[i];
    if (!ll) continue;

    const d = haversineM(rawLL, ll);
    if (d < bestDist) {
      bestDist = d;
      bestNode = i;
    }
  }

  if (bestNode < 0) {
    return { node: -1, snapped_ll: null, dist_m: Infinity, valid: false };
  }

  return {
    node: bestNode,
    snapped_ll: nodeCoordsLL[bestNode].slice(),
    dist_m: bestDist,
    valid: true,
  };
}

function buildPreviewState(rawLL) {
  const nearest = findNearestNode(rawLL);
  return {
    raw_ll: rawLL.slice(),
    node: nearest.node,
    snapped_ll: nearest.snapped_ll,
    snap_dist_m: nearest.dist_m,
    valid: nearest.valid,
  };
}

function buildCommittedStateFromPreview(preview) {
  /// Callers must not commit an invalid preview; snapped_ll is null when valid === false.
  if (!preview.valid) throw new Error("Cannot commit an invalid preview");
  return {
    raw_ll: preview.snapped_ll.slice(),
    node: preview.node,
    snapped_ll: preview.snapped_ll.slice(),
    snap_dist_m: 0,
    valid: true,
  };
}
