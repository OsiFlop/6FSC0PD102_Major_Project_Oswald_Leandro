[Back to README](../README.md)

# 1. Using the Plugin

The Route Planner Plugin calculates aircraft-aware routes inside Unreal Engine.
It does not only connect a start point and a target point with a straight line.
Instead, it checks whether a route is flyable with the selected aircraft profile.

The current system is built around a UI-driven route planner:

1. Terrain height data is baked into a grid.
2. A `RoutePlannerManager` receives UI input, marker positions, altitude values, and the selected aircraft profile.
3. A `FlightPathfinderActor` calculates the route.
4. The result is returned as a `RouteCalculationResult` for UI feedback and route drawing.

---

## Main Runtime Flow

The recommended route calculation flow is:

1. Select or create a `FlightProfile` data asset.
2. Prepare the terrain grid with `TerrainReferenceActor`, `VoxelGridBaker`, `VoxelGridConfig`, and `VoxelHeightCache`.
3. Place a `FlightPathfinderActor` in the level.
4. Place a `RoutePlannerManager` in the level.
5. Assign the pathfinder, route markers, and available aircraft profiles to the manager.
6. From UI or Blueprint, call:
   - `SetSelectedFlightProfile()`
   - `SetStartMarkerWorldXY()`
   - `SetTargetMarkerWorldXY()`
   - `CalculateRouteFromMarkers()`
7. Read the returned `FRouteCalculationResult`.

The manager is now the preferred entry point for UI-driven route planning.
The pathfinder can still be used directly in the editor for debugging.

---

## Terrain Preparation

The pathfinder needs baked terrain height data before it can calculate safe
routes. The terrain system consists of these parts:

| Component | Purpose |
|---|---|
| `TerrainReferenceActor` | Stores or automatically finds the landscape actor and exposes its world bounds. |
| `VoxelGridConfig` | Data asset containing grid size, sampling, trace, and debug settings. |
| `VoxelGridBaker` | Builds the grid over the landscape and bakes maximum terrain height per grid cell. |
| `VoxelHeightCache` | Data asset storing the baked grid metadata and terrain heights. |
| `HeightQueryProbeActor` | Optional debug helper for checking cached terrain height at a world position. |

### Terrain Setup Workflow

1. Place a `TerrainReferenceActor` in the level.
2. Assign the landscape manually or call `AutoFindLandscape()`.
3. Create or assign a `VoxelGridConfig` data asset.
4. Create or assign a `VoxelHeightCache` data asset.
5. Place a `VoxelGridBaker` in the level.
6. Assign:
   - `TerrainRef`
   - `GridConfig`
   - `HeightCache`
7. Call `BuildGrid()`.
8. Call `BakeMaxHeights()`.
9. Save the updated `VoxelHeightCache` asset.

The `VoxelHeightCache` is used by the pathfinder to compare aircraft altitude
against terrain altitude.

---

## VoxelGridConfig

`VoxelGridConfig` controls how the terrain grid is generated and sampled.

| Setting | Default | Description | Effect |
|---|---:|---|---|
| `CellSizeMeters` | `100.0` | Horizontal grid cell size. | Smaller values improve terrain precision but create more cells and more pathfinding work. |
| `PaddingMeters` | `0.0` | Extra border around the landscape bounds. | Larger values extend the searchable map area. |
| `SamplesPerAxis` | `3` | Number of height samples per cell axis. | Higher values detect terrain more accurately but increase bake time. |
| `TraceStartAboveMeters` | `3000.0` | Trace start height above the landscape actor. | Must be high enough to start above the terrain. |
| `TraceEndBelowMeters` | `3000.0` | Trace depth below the landscape actor. | Must be deep enough to hit low terrain. |
| `TraceChannel` | `ECC_Visibility` | Collision channel used for terrain tracing. | Controls which objects are hit while baking. |
| `DebugDrawLifetime` | `10.0` | Lifetime of grid debug lines. | Affects editor visualization only. |
| `DebugLineThickness` | `2.0` | Thickness of grid debug lines. | Affects editor visualization only. |

---

## VoxelHeightCache

`VoxelHeightCache` stores the baked terrain result.

| Setting | Description |
|---|---|
| `GridMinWorld` | Minimum world-space corner of the grid. |
| `GridSize` | Number of grid cells in X and Y. |
| `CellSizeCm` | Cell size in Unreal centimeters. |
| `MaxHeightCm` | Maximum terrain height per cell in world Z centimeters. |
| `SeaLevelWorldZCm` | World Z value used as sea-level reference for ASL conversion. |

`SeaLevelWorldZCm` is important because UI altitude values are entered in
meters ASL. The route planner converts them into Unreal world Z using this
value.

---

## Flight Profile

`FlightProfile` is a data asset containing aircraft-specific performance and
safety limits. Different aircraft should use different profiles.

### Aircraft Settings

| Setting | Description | Effect |
|---|---|---|
| `AircraftName` | Display name of the aircraft profile. | Used for UI/debug identification. |
| `MaxAltitudeMetersASL` | Maximum allowed altitude above sea level. | Route points above this altitude are rejected. |
| `CruiseSpeedMetersPerSecond` | Normal cruise speed. | Used for travel time, climb/descent validation, and preferred terrain clearance. |
| `MaxClimbRateMetersPerSecond` | Maximum climb speed. | Limits how quickly the route may climb. |
| `MaxDescentRateMetersPerSecond` | Maximum descent speed. | Limits how quickly the route may descend. |
| `RangeMeters` | Aircraft range. | Stored in the profile, but not currently used as a route rejection rule. |
| `StallSpeedMetersPerSecond` | Minimum safe speed. | Stored in the profile, but not currently used by the route search. |
| `MinimumTurnRadiusMeters` | Smallest safe turn radius. | Forces wider turns and affects motion primitive length. |

### Safety Settings

| Setting | Description | Effect |
|---|---|---|
| `MinimumTerrainClearanceMeters` | Minimum required height above terrain. | The route must stay at least this far above terrain. |

---

## RoutePlannerManager

`RoutePlannerManager` is the UI-facing route planner actor. It stores the
available aircraft profiles, the currently selected aircraft, the route markers,
and the latest route result.

### References and Data

| Setting | Description | Effect |
|---|---|---|
| `AvailableFlightProfiles` | List of selectable aircraft profiles. | Used by the UI to offer aircraft choices. |
| `SelectedFlightProfile` | Currently active profile. | Used for the next route calculation. |
| `PathfinderActor` | Actor that performs the actual search. | Must point to the `FlightPathfinderActor`. |
| `StartMarker` | Actor used as route start point. | Provides start X/Y position. |
| `TargetMarker` | Actor used as route target point. | Provides target X/Y position. |
| `MapWorldMin` | Minimum world-space XY corner of the map UI. | Used for map coordinate conversion. |
| `MapWorldMax` | Maximum world-space XY corner of the map UI. | Used for map coordinate conversion. |
| `CurrentRoutePoints` | Latest route points in world space. | Used by UI/visualization after success. |
| `LastResult` | Latest route result. | Stores success state, route points, failure reason, and debug count. |
| `bShowWeather` | UI option for weather overlay or weather-aware behavior. | Currently stored as an option; weather routing is not implemented in C++. |
| `bShowRestrictedZones` | UI option for restricted zone display/routing. | Stored on the manager; actual zone checks happen in the pathfinder. |

### Blueprint Functions

| Function | Description |
|---|---|
| `SetSelectedFlightProfile(NewProfile)` | Sets the active aircraft profile, clears the previous route, and syncs the profile to the pathfinder. |
| `CalculateRouteFromMarkers(StartAltitudeMetersASL, TargetAltitudeMetersASL)` | Calculates a route using the marker XY positions and the requested ASL altitudes. |
| `ClearCurrentRoute()` | Clears the manager route result. |
| `SetStartMarkerWorldXY(WorldXY)` | Moves the start marker in X/Y while keeping its current Z. |
| `SetTargetMarkerWorldXY(WorldXY)` | Moves the target marker in X/Y while keeping its current Z. |
| `GetStartMarkerLocation()` | Returns the current start marker world location. |
| `GetTargetMarkerLocation()` | Returns the current target marker world location. |

---

## FlightPathfinderActor

`FlightPathfinderActor` performs the actual route search.
It uses a 3D search space and includes the aircraft heading direction in the
search state. This allows the route to respect aircraft movement more closely
than a simple point-to-point voxel search.

The pathfinder now supports:

- UI route requests through `CalculateFlightRouteForUI()`
- editor route requests through `FindFlightRoute()`
- direct VFR route shortcut before A*
- motion primitives for aircraft-like movement
- climb/descent validation
- turn radius validation
- terrain clearance validation
- hard block zones and soft cost zones
- route compaction after search
- route result and failure text for UI

---

## Required Pathfinder References

| Setting | Description | Effect |
|---|---|---|
| `GridBaker` | Reference to the terrain grid baker. | Can provide the height cache if it is not assigned directly. |
| `HeightCache` | Baked terrain height data. | Required for terrain clearance and ASL conversion. |
| `FlightProfile` | Aircraft limits and safety values. | Required for flight validation. |
| `StartActor` | Route start position for editor calls. | Used by `FindFlightRoute()`. |
| `GoalActor` | Route target position for editor calls. | Used by `FindFlightRoute()`. |
| `InfluenceZones` | Hard block or soft cost areas. | Used to reject or penalize route segments. |

For UI-driven routing, `StartActor` and `GoalActor` are not the main input.
The manager passes marker locations and altitude values into
`CalculateFlightRouteForUI()`.

---

## Search Space Settings

| Setting | Default | Description | Effect |
|---|---:|---|---|
| `VoxelSizeZMeters` | `25.0` | Height of one vertical search layer. | Smaller values improve altitude precision but increase the number of layers. |
| `ExtraBottomLayers` | `0` | Extra layers below relevant terrain/route height. | Usually low for aircraft routing. |
| `ExtraTopLayers` | `8` | Extra layers above relevant terrain/route height. | Allows more climb options but increases search size. |
| `HeuristicWeight` | `1.0` | Weight applied to the A* heuristic. | Higher values can make search faster but less optimal. |
| `SearchMaxExpandedStates` | `1500000` | Maximum A* states expanded before aborting. | Higher values allow harder routes but increase worst-case time. |
| `GoalConnectionToleranceMeters` | `900.0` | Distance where direct connection to the exact goal may be accepted. | Higher values may finish the route earlier. |
| `DirectRoutePreferredClearanceMaxLengthMeters` | `0.0` | Maximum length for direct preferred-clearance route check. | `0` means no extra maximum length rule. |

### Generated Search Space Values

| Setting | Description |
|---|---|
| `SearchMinWorld` | Minimum world-space bound of the generated search space. |
| `SearchMaxWorld` | Maximum world-space bound of the generated search space. |
| `ZLayerCount` | Number of vertical search layers. |

---

## Flight Model Settings

| Setting | Default | Description | Effect |
|---|---:|---|---|
| `HeadingBucketCount` | `16` | Number of possible flight directions. | Higher values allow finer heading changes but create more states. |
| `MaxHeadingChangePerStep` | `1` | Maximum heading change per search step. | Higher values allow sharper turns. |
| `bAllowClimbStep` | `true` | Allows upward movement. | If disabled, the search cannot climb. |
| `bAllowDescentStep` | `true` | Allows downward movement. | If disabled, the search cannot descend. |
| `PrimitiveSegmentLengthMeters` | `400.0` | Preferred length of one aircraft motion primitive. | Longer primitives create larger movement steps. |
| `PrimitiveSamplesPerSegment` | `8` | Samples used to validate one primitive. | Higher values check curves and climbs more thoroughly. |
| `PrimitiveClimbRateFactor` | `0.85` | Safety factor for climb/descent inside a primitive. | Lower values make vertical movement more conservative. |
| `MaxAutoPrimitiveSegmentLengthMeters` | `2500.0` | Maximum automatically extended primitive length. | Prevents very long movement steps. |
| `PrimitiveTurnDeltaBuckets` | `1` | Heading change used by primitive turns. | Higher values allow stronger primitive turns. |

The effective primitive length can become longer than
`PrimitiveSegmentLengthMeters` if the selected aircraft needs more horizontal
distance for its turn radius or climb/descent limits.

---

## Safety Settings

| Setting | Default | Description | Effect |
|---|---:|---|---|
| `MinimumAbsoluteTerrainClearanceMeters` | `50.0` | Project-wide minimum terrain clearance. | The required clearance is at least this value even if the profile is lower. |
| `TerrainClearanceSafetyMultiplier` | `1.5` | Multiplier for preferred terrain clearance. | Higher values make the cost system prefer safer/higher routes. |
| `LateralTerrainSafetyRadiusMultiplier` | `1.0` | Multiplier for lateral terrain safety sampling. | Higher values make terrain checks consider a wider area. |
| `PreferredClearanceSpeedLookaheadSeconds` | `6.0` | Speed-based extra clearance lookahead. | Faster aircraft receive more preferred clearance buffer. |

The system uses two clearance concepts:

| Clearance Type | Meaning |
|---|---|
| Required clearance | Hard rule. If this is violated, the route is rejected. |
| Preferred clearance | Soft preference. Routes closer to terrain can become more expensive even if still legal. |

---

## Influence Zones

`FlightInfluenceZoneActor` defines airspace zones with a box volume and optional
altitude limits.

| Setting | Description | Effect |
|---|---|---|
| `ZoneBox` | 3D box volume of the zone. | Defines the affected world area. |
| `bHardBlock` | Whether the zone is forbidden. | `true` rejects routes through the zone. |
| `AdditionalTraversalCost` | Extra cost for soft zones. | Used only when `bHardBlock` is `false`. |
| `MinAltitudeMetersASL` | Lower active altitude. | Points below this height ignore the zone. |
| `MaxAltitudeMetersASL` | Upper active altitude. | If lower than min altitude, no upper limit is applied. |

Hard block zones are rejected during point and segment checks.
Soft cost zones are allowed, but the A* cost is increased so the route will
prefer avoiding them when possible.

---

## Result Settings and Debug Data

| Setting | Default | Description | Effect |
|---|---:|---|---|
| `MinOutputWaypointSpacingMeters` | `750.0` | Minimum spacing after waypoint compaction. | Higher values produce fewer output waypoints. |
| `MaxOutputWaypointCompactionLookahead` | `16` | Maximum points tested ahead during compaction. | Higher values can simplify more aggressively. |
| `CurrentRouteWorldPoints` | generated | Final route in world-space coordinates. | Used for drawing, UI, or later aircraft following logic. |
| `LastExpandedStates` | generated | Number of A* states expanded in the last search. | Useful for performance debugging. |
| `FailureStats` | generated | Counts why candidate states or transitions failed. | Useful for diagnosing failed searches. |
| `LastFailureReason` | generated | Last high-level route failure reason. | Used for UI failure text. |

---

## Debug Settings

| Setting | Default | Description | Effect |
|---|---:|---|---|
| `bDrawVisitedStates` | `false` | Draws checked A* states. | Useful for debugging but can be expensive. |
| `bAutoDrawPathAfterSearch` | `true` | Draws the route after a successful search. | Makes successful routes immediately visible. |
| `DebugDrawLifetime` | `20.0` | Lifetime of debug shapes. | Controls how long debug visuals remain visible. |
| `DebugLineThickness` | `6.0` | Route line thickness. | Higher values make the path easier to see. |
| `DebugVisitedPointSize` | `16.0` | Size of visited-state debug points. | Higher values make visited states easier to see. |

---

## Route Result

`FRouteCalculationResult` is returned to UI and Blueprint callers.

| Field | Description |
|---|---|
| `bSuccess` | Whether the route calculation succeeded. |
| `RoutePoints` | Route points in world space. Empty when the route fails. |
| `FailureReason` | Technical failure reason as `ERouteFailureReason`. |
| `FailureText` | User-facing text explaining the failure. |
| `ExpandedStates` | Number of A* states expanded during the calculation. |

Common failure reasons include invalid start/target, invalid flight profile,
altitude too low/high, terrain clearance violation, hard block zone, climb or
descent limit, turn radius limit, and search limit reached.

---

## Editor Functions

### FlightPathfinderActor

| Function | Description |
|---|---|
| `BuildSearchSpace()` | Builds the 3D search space from the terrain cache. Mostly useful for editor/debug workflows. |
| `FindFlightRoute()` | Runs route search using `StartActor`, `GoalActor`, and `FlightProfile`. |
| `DebugDrawCurrentRoute()` | Draws the current route in the editor viewport. |
| `ClearCurrentRoute()` | Clears route points and debug drawings. |
| `CalculateFlightRouteForUI()` | Blueprint-callable route calculation using explicit positions, altitudes, and profile. |
| `GetFailureReasonText()` | Converts a failure enum into readable UI text. |

### VoxelGridBaker

| Function | Description |
|---|---|
| `BuildGrid()` | Builds the horizontal terrain grid from landscape bounds. |
| `BakeMaxHeights()` | Stores maximum terrain height per grid cell in `VoxelHeightCache`. |
| `DebugDrawGridOutline()` | Draws the grid boundary. |
| `DebugDrawSomeCells()` | Draws a limited number of grid cells. |
| `BuildPreviewVoxels()` | Builds a voxel-column preview from cached terrain height. |
| `ClearPreviewVoxels()` | Removes the preview voxel instances. |

### RoutePlannerManager

| Function | Description |
|---|---|
| `SetSelectedFlightProfile()` | Sets and syncs the active profile. |
| `CalculateRouteFromMarkers()` | Main UI route calculation function. |
| `ClearCurrentRoute()` | Clears the stored result. |
| `SetStartMarkerWorldXY()` | Moves the start marker from map input. |
| `SetTargetMarkerWorldXY()` | Moves the target marker from map input. |

---

## Recommended UI Workflow

Use this workflow when calculating routes from the route planner UI:

1. Make sure the terrain grid is built and the `VoxelHeightCache` is valid.
2. Place and configure a `FlightPathfinderActor`.
3. Place and configure a `RoutePlannerManager`.
4. Assign the manager references:
   - `PathfinderActor`
   - `StartMarker`
   - `TargetMarker`
   - `AvailableFlightProfiles`
5. Assign the pathfinder references:
   - `GridBaker`
   - `HeightCache`
   - `InfluenceZones` if needed
6. Select an aircraft profile through `SetSelectedFlightProfile()`.
7. Move markers with `SetStartMarkerWorldXY()` and `SetTargetMarkerWorldXY()`.
8. Call `CalculateRouteFromMarkers(StartAltitudeMetersASL, TargetAltitudeMetersASL)`.
9. If `bSuccess` is true, draw or use `RoutePoints`.
10. If `bSuccess` is false, display `FailureText`.

---

## Recommended Editor Debug Workflow

Use this workflow when testing the pathfinder directly in the editor:

1. Place `StartActor` and `GoalActor`.
2. Assign them to the `FlightPathfinderActor`.
3. Assign `GridBaker`, `HeightCache`, and `FlightProfile`.
4. Optionally assign `InfluenceZones`.
5. Call `FindFlightRoute()`.
6. Inspect the drawn route, `LastFailureReason`, `FailureStats`, and `LastExpandedStates`.

Calling `BuildSearchSpace()` manually is useful for debugging, but the UI route
calculation builds a route-specific search space internally.

---

## Performance Notes

These settings have the strongest performance impact:

| Setting | Performance Impact |
|---|---|
| Smaller `CellSizeMeters` in `VoxelGridConfig` | More terrain cells and more pathfinding positions. |
| Smaller `VoxelSizeZMeters` | More vertical layers and more search states. |
| Higher `ExtraTopLayers` | More vertical search options. |
| Higher `HeadingBucketCount` | More directional states. |
| Higher `MaxHeadingChangePerStep` or `PrimitiveTurnDeltaBuckets` | More neighbor candidates per state. |
| Higher `SearchMaxExpandedStates` | Larger worst-case search. |
| `bDrawVisitedStates = true` | Can slow the editor heavily on large searches. |
| Higher `PrimitiveSamplesPerSegment` | More safety checks per movement. |

For faster searches:

- Increase `VoxelSizeZMeters`.
- Keep `HeadingBucketCount` moderate.
- Keep `ExtraTopLayers` as low as possible.
- Disable `bDrawVisitedStates`.
- Keep grid cell size practical for the map scale.

For safer and more realistic routes:

- Use accurate aircraft profiles.
- Keep a meaningful `MinimumTerrainClearanceMeters`.
- Use a realistic `MinimumTurnRadiusMeters`.
- Use enough primitive samples to catch terrain and zone intersections.
- Use hard block zones for forbidden airspace.

---

## Current Limitations

- `RangeMeters` and `StallSpeedMetersPerSecond` are stored in `FlightProfile`, but they are not currently used as hard route validation rules.
- `bShowWeather` is stored on `RoutePlannerManager`, but weather-aware routing is not currently implemented in C++.
- The main route planner uses baked terrain height data. The cache must be rebuilt when the terrain changes.
- The UI workflow should use `RoutePlannerManager`; direct `FindFlightRoute()` is mainly for editor debugging.
