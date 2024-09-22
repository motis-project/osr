<p align="center"><img src="docs/screenshot.png"></p>

Demo (currently limited to 1h radius): https://osr.motis-project.de

# Open Street Router

This router is the most memory-efficient multi-profile routing for planet wide street routing (pedestrian, bike, car, etc.) on OpenStreetMap. The
goal is to make it possible to import data on affordable low-end machines. This is mainly achieved by using compact data
structures and [memory mapped](https://en.wikipedia.org/wiki/Memory-mapped_file) files. A planet import should not need
more than 10GB of RAM. More RAM (and a fast SSD) will speed up the import.

Directory with all created files after extract (only routing data has to be in-memory):

```bash
# routing data 12.7G
# recommended to lock to memory
 22K multi_level_elevators.bin
1,2G node_in_way_idx_data.bin
1,1G node_in_way_idx_index.bin
542M node_properties.bin
 34M node_restricted.bin
4,7M node_restrictions_data.bin
1,1G node_restrictions_index.bin
2,3G node_ways_data.bin
1,1G node_ways_index.bin
748M way_node_dist_data.bin
833M way_node_dist_index.bin
2,3G way_nodes_data.bin
833M way_nodes_index.bin
625M way_properties.bin

# mapping to osm ids
2,2G node_to_osm.bin
1,7G way_osm_idx.bin

# way osm nodes (only memory mapped)
 19G way_osm_nodes_data.bin
1,7G way_osm_nodes_index.bin

# way geometry (only memory mapped)
 19G way_polylines_data.bin
1,7G way_polylines_index.bin
```

## Multi-Level Indoor Routing

<p align="center"><img src="docs/levels.png"></p>

## Car Outdoor Routing

<p align="center"><img src="docs/car.png"></p>

## Usage

```bash
# --in     | -i     input file
# --out    | -o     output directory (will be deleted + created)
./osr-extract -i planet-latest.osm.pbf -o osr-planet

# --data   | -d     the output from osr-extract
# --static | -s     static HTML/JS/CSS assets to serve
./osr-backend -d osr-planet -s web
```

## Data Model

<p align="center"><img src="docs/data_model.png"></p>

### Multi Nodes

The data model is not an explicit graph. Instead, the OpenStreetMap data (nodes and ways) is stored more or less as it
is. For routing purposes (i.e. finding shortest paths), only nodes are relevant that are part of more than one way.
Those nodes are given special IDs (`node_idx_t` in contrast to `osm_node_idx_t` which is the node index from
OpenStreetMap).

### Lookup

Since only a fraction of ways in OpenStreetMap are relevant for routing, we give the extracted ways internal
indices (`way_idx_t` in contrast to `osm_way_idx_t` which is the way index from OpenStreetMap). To be able to map
from `node_idx_t` to `osm_node_idx_t` and from `way_idx_t` to `osm_way_idx_t` and vice versa, we have a lookup table in
both directions.

### Way to Node and Node to Way

For routing, it's important to have a fast way to know which ways are reachable from which nodes and which nodes are
reachable from which way. To achieve this, we store for each `way_idx_t` a list of `node_idx_t` and for
each `node_idx_t` a list of `way_idx_t` coupled with the information which index this node has in the corresponding way.
If a node is part of a way multiple times, this mapping will contain the node multiple times. The order from the source
way in OpenStreetMap is maintained.

## Routing

Dijkstra`s algorithm with a time-based cutoff is used currently for routing.

### Node Neighborhood

For routing (here: Dijkstra's algorithm for now), it's necessary to know the neighborhood of a node. This can be looked
up by iterating all `way_idx_t` this node is contained in (see above) and the corresponding index `i` this node has in
the way. Neighbors are then node indices `i-1` and `i+1` in those ways. For the case that `i=0` or points to the last
index, `i-1` or `i+1` do not exist. With the definition of this neighborhood, the textbook version of Dijkstra's
algorithm can be applied.

### From Coordinate to Graph

To be able to resolve a geo coordinate into nodes in the graph for routing, a geo index data like a quad tree or r-tree
is required. As there are less `way_idx_t` than `node_idx_t`, we store the bounding boxes of all ways into the rtree. A
lookup gives us all `way_idx_t` in the area. Sorting those ways by perpendicular line distance from the query coordinate
to the way gives us the closes `way_idx_t` for start and destination. After that, the two closest routing nodes ("left"
and "right") on that `way_idx_t` can be initialized.

### Routing Profiles

All attributes of the OpenStreetMap ways that are relevant for routing are packed into a compact struct. Distances
between routing nodes (i.e. `node_idx_t`) on a way are stored for fast access.
The profile (see blow) function then computes the edge weights and feasibility of a way based on the stored OpenStreetMap attributes.
To implement more finegrained control over path finding, it might be necessary to extract more attributes from OpenSteetMap.

The current set of routing profiles can be found in the [`include/osr/routing/profiles`](https://github.com/motis-project/osr/tree/master/include/osr/routing/profiles) folder. Here's what a profile needs to provide in order to be usable by the shortest path algorithm:

#### Types

- `node` struct: a node defines what is considered a node for this routing profile. The most basic definition of a node would basically be just what's considered a node in the data model (`node_idx_t`, see above). However, for many profiles, this is not succifient. Let's take the foot routing profile as an example: for indoor routing, it should be possible to distinguish the same OSM node on different levels. Therefore, the level has to be part of the foot profile's node definition. Another example is the car profile: to be able to detect u-turns (just changing the direction but staying on the same way), the node has to define the current direction. In order to properly handle turn restrictions, also the last used way has to be part of the node definition. The member function `node::get_key()` returns the key (see `key`).
- `label` struct: basically the same as the `node` struct but it should additionally carry the routing costs (currently tracked in seconds). The label has to provide `get_node()` and `cost()`  member functions.
- `key`: The shortest path algorithm (e.g. Dijkstra) tracks minimal costs to each node in a hash map. In theory, it would be sufficient to use `node` as hash map key as we need to only track one shortest path to each profile `node`. To allow for a more efficient storage, multiple nodes can share the same hash map key, therefore reducing the hash map size (which can speedup the routing significantly). Therefore, a profile can define a `key` which can be the same as `node` but doesn't have to be. The key doesn't need any member functions and can therefore just be a typedef to `node_idx_t` (in the most simple case).
- `entry`: The entry is the hash map entry and is stored for each `key`. It has to store the costs and precessor. If `key` maps several `node`s to the same `entry`, then the `entry` has to store costs and predecessory for each of the `node`s. It has to provide the following member functions:
  - `std::optional<node> pred(node)`: returns the predecessor for the node, or `std::nullopt` if this `node` has never been visited.
  - `cost_t cost(node)`: returns the shortest path costs to this `node`
  - `bool update(label, node, cost_t, node pred)`: updates the costs for this node if they are better than the previously stored costs and returns `true` if the costs where updated and `false` otherwise.

#### Static functions

  - `resolve_start_node(ways::routing, way_idx_t, node_idx_t, level_t, direction, Fn&& f)`: resolves all nodes that belong to this particular (`way_idx_t`, `node_idx_t`, `level_t`, `direction`) combination. `Fn f` will be called with each `node`. It's the task of the profile to give the routing algorithm and entry point to its overlay graph.
  - `resolve_all(ways::routing, node_ix_t, level_t, Fn&& f)`: Same as `resolve_start_node`, just without the condition that `way_idx_t` has to match.
  - `adjacent<SearcHdir, WithBlocked, Fn>(ways::routing, node, bitvec<node_idx_t> blocked, Fn&& f)`: Calls `Fn f` with each adjacent neighbor of the given `node`. This is used in the shortest path algorithm to expand a node and visit all its neighbors. This takes a runtime provided bit vector `blocked` into account where bit `i` indicates if `i` can be visited or not. This allows us to dynamically block nodes depending on the routing query.

As we can see, each profile can define its own overlay graph on top of the data model. This gives us the flexibility to define a routing for anything we want from pedestrians or wheelchair users over cars, trucks, trains to ships without any additional memory overhead. Even combined profiles (e.g. walking, taking a bike, walking) can be implemented. Commonly, routing engines have to have a graph for each profile which makes it quite expensive (in terms of memory) to add a new profile on a global routing server. With our approach, a new profile doesn't come with extra costs.

Profiles have to be defined at compile time. However, it would be possible to parameterize the cost calculation (or any other aspect, like restrictions, etc.) at runtime.


### Reconstruction

Reconstruction works as in the textbook version of Dijkstra's algorithm except for the first and last part of the path
which is not part of the routing "graph" (geo coordinate to first `node_idx_t` and last `node_idx_t` to the geo
coordinate).

## Future Work

This is only a first proof-of-concept. Many basic as well as advanced features can follow.

Known Issues:

- Routing performance can be improved
  - by using A* or bidirectional A* for one to one queries
  - explore preprocessing-based approaches: landmarks, arc flags, transit node routing, multi-level-dijkstra, etc.
- If source and target are mapped to the same way, the path should not be forced to go through routing nodes
- Consider the routing profile for initialization

Basic:

- Extract street names of ways for reconstruction.
- Reconstruct description of way for navigation (including street names)
- Turn restriction relations ([example](https://www.openstreetmap.org/relation/1654115), [example](https://www.openstreetmap.org/node/516914))
- Penalize u-turns

Advanced:

- Create a compact memory-mapped or serializable version of [rtree.c](https://github.com/tidwall/rtree.c)
- Enable routing algorithm to have weights for nodes, not just edges (e.g. elevators, street crossings, etc.)
- Incremental live update with OpenStreetMap change sets (should be doable as the data model is not far away from OSM)
- Exclusion zones as part of the routing query (e.g. for e-scooter routing)
- Time-dependent routing (consider traffic flow forecast and/or live traffic)
- Add height profile information to edges (relevant e.g. for bike routing) and use it in routing profiles
- Combined routing for sharing mobility (e.g. walk to e-scooter, ride e-scooter, walk to destination)
- `vehicle=destination` ([example](https://www.openstreetmap.org/way/61914850))
- Start heading + destination heading
- Make barriers / inaccessible nodes routing nodes (example: [way](https://www.openstreetmap.org/way/940718404), [node](https://www.openstreetmap.org/node/8712182900))
