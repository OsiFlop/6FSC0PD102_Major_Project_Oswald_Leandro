[Back to README](../README.md)

# 2. Plugin Structure

> This README explains the main scripts used by the route planner system.
>
> **Note:** The lists below focus on values that are stored permanently, such as `UPROPERTY` fields, struct fields, enum values, and private cache fields. Local helper variables inside individual methods are not listed separately because they only exist temporarily inside a method.

---

## Overall Purpose

The route planner calculates a flight route from a start point to a target point. It does not simply draw a straight line. Instead, it searches for a safe route and checks several flight rules during the process.

The pathfinder checks:

- terrain clearance
- maximum flight altitude
- climb rate
- descent rate
- turn radius
- restricted zones
- high-cost influence zones

## Basic Workflow

1. The UI communicates with the `RoutePlannerManager`.
2. The manager reads the start marker, target marker, altitude values, and selected `FlightProfile`.
3. The manager calls the `FlightPathfinderActor`.
4. The `FlightPathfinderActor` uses the `VoxelHeightCache`, `FlightProfile`, and `InfluenceZones`.
5. First, it tries to create a direct VFR route. If that route is not safe, it starts an A* search in a 3D grid.
6. The result is returned to the UI and the manager as an `FRouteCalculationResult`.

## Important Script Collaboration

- `TerrainReferenceActor` provides the landscape and its bounds.
- `VoxelGridBaker` creates a grid from the landscape and bakes the height data.
- `VoxelHeightCache` stores the terrain height for each grid cell.
- `FlightProfile` describes the aircraft limits.
- `FlightInfluenceZoneActor` describes blocked or expensive areas.
- `FlightPathfinderActor` calculates the route.
- `RoutePlannerManager` connects the UI, markers, selected profile, and pathfinder.

---

# 1. RoutePlannerManager.h / RoutePlannerManager.cpp

## Name

`RoutePlannerManager`

## Main Purpose

The `RoutePlannerManager` is the central connection between the UI, map markers, aircraft selection, and the pathfinder.

It stores the current selection and the last calculated route. When the UI requests a route calculation, the manager calls the `FlightPathfinderActor`.

## Communication

- Uses `UFlightProfile` for aircraft profiles.
- Uses `AFlightPathfinderActor` for the actual route calculation.
- Uses `FRouteCalculationResult` for success state, error messages, and route data.
- Uses `StartMarker` and `TargetMarker` as world positions.

## Variables

| Variable | Type | Purpose |
|---|---|---|
| `AvailableFlightProfiles` | `TArray<TObjectPtr<UFlightProfile>>` | List of selectable aircraft profiles. Changes which aircraft are available in the UI. |
| `SelectedFlightProfile` | `TObjectPtr<UFlightProfile>` | Active aircraft profile. Changes which aircraft limits are used for the next route. |
| `PathfinderActor` | `TObjectPtr<AFlightPathfinderActor>` | Reference to the pathfinder actor. Changes which actor calculates the route. |
| `StartMarker` | `TObjectPtr<AActor>` | Actor used as the route start point. Changes the start of the route. |
| `TargetMarker` | `TObjectPtr<AActor>` | Actor used as the route target point. Changes the destination of the route. |
| `MapWorldMin` | `FVector2D` | Lower XY world corner of the map. Changes the map scale or visible map area. |
| `MapWorldMax` | `FVector2D` | Upper XY world corner of the map. Changes the map scale or visible map area. |
| `CurrentRoutePoints` | `TArray<FVector>` | Last route as world points. Changes which route is displayed. |
| `LastResult` | `FRouteCalculationResult` | Last calculation result. Changes status, error text, and displayed result values. |
| `bShowWeather` | `bool` | UI option for weather display or weather-aware logic. In the current C++ code, this is only stored and not yet actively used in the search. |
| `bShowRestrictedZones` | `bool` | UI option for restricted zones. The actual restricted-zone logic is handled in `FlightPathfinderActor`. |

## Methods

| Method | Purpose |
|---|---|
| `ARoutePlannerManager()` | Constructor. Disables tick. |
| `SetSelectedFlightProfile(UFlightProfile* NewProfile)` | Sets the active profile, clears old route data, passes the profile to the pathfinder, and clears the pathfinder route. |
| `CalculateRouteFromMarkers(float StartAltitudeMetersASL, float TargetAltitudeMetersASL)` | Checks pathfinder, markers, and profile, then calls `CalculateFlightRouteForUI()` in the pathfinder and stores the result. |
| `ClearCurrentRoute()` | Clears `CurrentRoutePoints` and resets `LastResult`. |
| `SetStartMarkerWorldXY(FVector2D WorldXY)` | Moves the start marker in X/Y while keeping the current Z value. |
| `SetTargetMarkerWorldXY(FVector2D WorldXY)` | Moves the target marker in X/Y while keeping the current Z value. |
| `GetStartMarkerLocation() const` | Returns the start marker location, or `ZeroVector` if no marker is set. |
| `GetTargetMarkerLocation() const` | Returns the target marker location, or `ZeroVector` if no marker is set. |

---

# 2. RouteCalculationResult.h

## Name

`RouteCalculationResult`

## Main Purpose

Defines the struct `FRouteCalculationResult`. This struct is the return format of a route calculation.

## Communication

- Created by `FlightPathfinderActor`.
- Stored by `RoutePlannerManager`.
- Passed to the UI.
- Uses `ERouteFailureReason`.

## Variables

| Variable | Type | Purpose |
|---|---|---|
| `bSuccess` | `bool` | Shows if a route was found. `true` means success, `false` means failure. |
| `RoutePoints` | `TArray<FVector>` | Calculated route waypoints. Changes the route that is drawn or flown. |
| `FailureReason` | `ERouteFailureReason` | Technical failure reason. Changes which type of error occurred. |
| `FailureText` | `FText` | Readable error text. Changes the message shown in the UI. |
| `ExpandedStates` | `int32` | Number of checked A* states. Only affects debug or performance display. |

## Methods

This struct does not define its own methods. It only stores data.

---

# 3. RouteFailureReason.h

## Name

`RouteFailureReason`

## Main Purpose

Defines all failure reasons shared by the pathfinder, manager, and UI.

## Communication

- Stored in `FRouteCalculationResult`.
- Set by `FlightPathfinderActor`.
- Converted into readable text inside the pathfinder.

## Enum Values

| Value | Meaning |
|---|---|
| `None` | No error. |
| `InvalidStart` | Start point is invalid. |
| `InvalidTarget` | Target point is invalid. |
| `InvalidTargetState` | Target cannot be used as a safe search state. |
| `InvalidFlightProfile` | No valid aircraft profile is set. |
| `StartAltitudeTooLow` | Start altitude is too low or invalid. |
| `TargetAltitudeTooLow` | Target altitude is too low or invalid. |
| `TargetAltitudeTooHigh` | Target altitude is above the maximum flight altitude. |
| `TargetAltitudeNotReachable` | Target altitude cannot be reached by the aircraft. |
| `TerrainCollision` | Route collides with terrain. |
| `TerrainClearance` | Required terrain clearance is not met. |
| `RestrictedAirspace` | Route enters restricted airspace. |
| `HardBlockZone` | Route touches a hard blocked zone. |
| `ClimbLimitExceeded` / `MaxClimbExceeded` | Climb rate is too high. |
| `DescentLimitExceeded` / `MaxDescentExceeded` | Descent rate is too high. |
| `TurnRadiusExceeded` / `TurnRadiusTooSmall` | Turn is tighter than the aircraft limit. |
| `NoValidNeighbors` | No valid neighbor states were found. |
| `SearchLimitReached` / `MaxExpandedStatesReached` | Search limit was reached. |
| `Unknown` | General error without a more specific reason. |

## Methods

This file does not define methods. It only defines the enum.

---

# 4. FlightProfile.h / FlightProfile.cpp

## Name

`FlightProfile`

## Main Purpose

`FlightProfile` is a `DataAsset` for aircraft data. It stores performance and safety values used by the pathfinder.

## Communication

- Selected in `RoutePlannerManager`.
- Passed to `FlightPathfinderActor`.
- Also validated in `Simple3DPathfinderActor`.

## Variables

| Variable | Type | Purpose |
|---|---|---|
| `AircraftName` | `FString` | Display name of the profile. Changes the name shown in UI or logs. |
| `MaxAltitudeMetersASL` | `float` | Maximum altitude above sea level. Higher values allow higher routes. |
| `CruiseSpeedMetersPerSecond` | `float` | Normal flight speed. Affects flight time, climb/descent checks, and safety lookahead. |
| `MaxClimbRateMetersPerSecond` | `float` | Maximum climb rate. Higher values allow steeper climbs. |
| `MaxDescentRateMetersPerSecond` | `float` | Maximum descent rate. Higher values allow steeper descents. |
| `MinimumTerrainClearanceMeters` | `float` | Minimum distance to terrain. Higher values force safer or higher routes. |
| `RangeMeters` | `float` | Aircraft range. In the current pathfinder, this is not yet actively used as a limit. |
| `StallSpeedMetersPerSecond` | `float` | Minimum speed before stall. In the current pathfinder, this is not yet actively checked. |
| `MinimumTurnRadiusMeters` | `float` | Smallest safe turn radius. Higher values force wider turns. |

## Methods

This class does not define its own methods. `FlightProfile` is a pure `DataAsset`.

---

# 5. FlightInfluenceZoneActor.h / FlightInfluenceZoneActor.cpp

## Name

`FlightInfluenceZoneActor`

## Main Purpose

Defines 3D zones that influence the flight path. A zone can either be a hard blocked zone or a soft zone with additional traversal cost.

## Communication

- Referenced in `InfluenceZones` inside `FlightPathfinderActor`.
- The pathfinder uses `ContainsPoint()` and `IntersectsSegmentBySampling()`.

## Variables

| Variable | Type | Purpose |
|---|---|---|
| `ZoneBox` | `TObjectPtr<UBoxComponent>` | World volume of the zone. Changes the position and size of the zone. |
| `bHardBlock` | `bool` | `true` means forbidden, `false` means allowed but expensive. Changes whether the route is blocked or only penalized. |
| `AdditionalTraversalCost` | `float` | Additional cost for soft zones. Higher values make A* avoid the zone more strongly. |
| `MinAltitudeMetersASL` | `float` | Lower active altitude of the zone. Points below this altitude ignore the zone. |
| `MaxAltitudeMetersASL` | `float` | Upper active altitude of the zone. If `Max < Min`, no upper limit is applied. |

## Methods

| Method | Purpose |
|---|---|
| `AFlightInfluenceZoneActor()` | Creates `ZoneBox`, sets it as root, and disables physical collision. |
| `ContainsPoint(const FVector& WorldPoint, float PointAltitudeMetersASL) const` | Checks whether a point is inside the box and altitude range. |
| `IntersectsSegmentBySampling(...)` | Samples points along a segment and checks if one of them is inside the zone. |

---

# 6. AltitudeDebugActor.h / AltitudeDebugActor.cpp

## Name

`AltitudeDebugActor`

## Main Purpose

Debug helper. Converts the world Z values of start and goal into meters ASL and prints the values to the log.

## Communication

- Uses `UVoxelHeightCache` as sea-level reference.
- Uses `StartActor` and `GoalActor` as altitude sources.

## Variables

| Variable | Type | Purpose |
|---|---|---|
| `HeightCache` | `TObjectPtr<UVoxelHeightCache>` | Provides `SeaLevelWorldZCm`. Changes the ASL reference. |
| `StartActor` | `TObjectPtr<AActor>` | Actor used for the start altitude. Changes which point is measured. |
| `GoalActor` | `TObjectPtr<AActor>` | Actor used for the target altitude. Changes which point is measured. |

## Methods

| Method | Purpose |
|---|---|
| `AAltitudeDebugActor()` | Constructor. Disables tick. |
| `ValidateReferences() const` | Checks `HeightCache`, `StartActor`, and `GoalActor`. |
| `GetAltitudeMetersASLFromWorldZ(float WorldZCm) const` | Converts world Z into meters above sea level. |
| `PrintStartAndGoalASL()` | Prints start ASL, goal ASL, and the difference to the log. |

---

# 7. VoxelGridConfig.h / VoxelGridConfig.cpp

## Name

`VoxelGridConfig`

## Main Purpose

`DataAsset` containing settings for the grid, sampling, tracing, and debug drawing. The baker reads these values when building and baking the grid.

## Communication

- Used by `VoxelGridBaker`.

## Variables

| Variable | Type | Purpose |
|---|---|---|
| `CellSizeMeters` | `float` | XY size of one grid cell in meters. Smaller values are more accurate but create more cells and more work. |
| `PaddingMeters` | `float` | Extra border around the landscape. Higher values expand the grid area. |
| `DebugDrawLifetime` | `float` | Lifetime of debug lines. Changes how long lines stay visible. |
| `DebugLineThickness` | `float` | Thickness of debug lines. Changes visibility. |
| `SamplesPerAxis` | `int32` | Number of samples per axis inside one cell. Higher values detect terrain more accurately but require more traces. |
| `TraceStartAboveMeters` | `float` | Start point of line traces above the terrain. Must be high enough to hit terrain. |
| `TraceEndBelowMeters` | `float` | End point of line traces below the terrain. Must be low enough to hit deep areas. |
| `TraceChannel` | `TEnumAsByte<ECollisionChannel>` | Collision channel used for terrain traces. Changes which objects are hit. |

## Methods

This asset does not define its own methods. It only stores configuration values.

---

# 8. VoxelHeightCache.h / VoxelHeightCache.cpp

## Name

`VoxelHeightCache`

## Main Purpose

Stores baked terrain heights. For each grid cell, the maximum world Z height is saved. The pathfinder uses this data for fast terrain-clearance checks.

## Communication

- Written by `VoxelGridBaker`.
- Read by `FlightPathfinderActor`, `Simple3DPathfinderActor`, `HeightQueryProbeActor`, and `AltitudeDebugActor`.

## Variables

| Variable | Type | Purpose |
|---|---|---|
| `GridMinWorld` | `FVector` | Lower world position of the grid. Changes the origin of all grid queries. |
| `GridSize` | `FIntPoint` | Number of cells in X/Y. Changes grid size and array length. |
| `CellSizeCm` | `float` | Cell size in centimeters. Changes grid resolution. |
| `MaxHeightCm` | `TArray<float>` | Maximum terrain height per cell. Changes what counts as terrain obstacle. |
| `SeaLevelWorldZCm` | `float` | World Z position of sea level. Changes all ASL conversions. |

## Methods

| Method | Purpose |
|---|---|
| `Allocate(int32 SizeX, int32 SizeY)` | Sets `GridSize`, reserves `MaxHeightCm`, and initializes all values as invalid. |
| `IsValid() const` | Checks if cell size, grid size, and array length are consistent. |
| `ToIndex(int32 X, int32 Y) const` | Converts X/Y cell coordinates into an array index. |
| `GetHeightMetersASL(int32 X, int32 Y) const` | Returns the height of a cell in meters above sea level. |

---

# 9. VoxelGridBaker.h / VoxelGridBaker.cpp

## Name

`VoxelGridBaker`

## Main Purpose

Builds an XY grid over the landscape and bakes the maximum terrain height of each cell into the `VoxelHeightCache`. This creates the data base for later route calculation.

## Communication

- Uses `TerrainReferenceActor` for landscape bounds.
- Uses `VoxelGridConfig` for grid and sampling values.
- Writes into `VoxelHeightCache`.
- Optionally uses `HeightQueryProbeActor` as the preview center.

## Variables

| Variable | Type | Purpose |
|---|---|---|
| `TerrainRef` | `TObjectPtr<ATerrainReferenceActor>` | Landscape reference. Changes which terrain is rasterized. |
| `GridConfig` | `TObjectPtr<UVoxelGridConfig>` | Grid and sampling configuration. Changes accuracy and debug values. |
| `HeightCache` | `UVoxelHeightCache*` | Target for baked height data. Changes where data is written. |
| `GridMinWorld` | `FVector` | Lower world border of the grid. Changes the grid origin. |
| `GridMaxWorld` | `FVector` | Upper world border of the grid. Changes the grid size. |
| `GridSize` | `FIntPoint` | Number of cells in X/Y. Changes performance cost and accuracy. |
| `CellSizeCm` | `float` | Cell size in centimeters. Changes resolution. |
| `PreviewCenterActor` | `AHeightQueryProbeActor*` | Center point for the preview. Changes where the preview is placed. |
| `PreviewRadiusCells` | `int32` | Preview radius in cells. Changes preview size. |
| `bPreviewUseSeaLevelAsBase` | `bool` | Uses sea level as the base of preview columns. Changes the bottom of the visualization. |
| `PreviewBaseZOverrideCm` | `float` | Manual base Z height. Changes the preview bottom when sea level is not used. |
| `PreviewMaterial` | `UMaterialInterface*` | Material for the preview cubes. Changes the visual appearance. |
| `PreviewHISM` | `UHierarchicalInstancedStaticMeshComponent*` | Internal instance component for preview cubes. Created and removed when building the preview. |

## Methods

| Method | Purpose |
|---|---|
| `AVoxelGridBaker()` | Constructor. Disables tick. |
| `IsGridValid() const` | Checks `GridSize` and `CellSizeCm`. |
| `GetCellMinMaxXY(int32 X, int32 Y, FVector2D& OutMin, FVector2D& OutMax) const` | Calculates the world bounds of one grid cell. |
| `BuildGrid()` | Reads landscape bounds, applies padding, and calculates `GridMinWorld`, `GridMaxWorld`, `GridSize`, and `CellSizeCm`. |
| `DebugDrawGridOutline()` | Draws the outer grid border. |
| `DebugDrawSomeCells(int32 MaxCellsToDraw)` | Draws a limited number of grid cells. |
| `DebugDrawSomeCells50()` | Convenience method. Calls `DebugDrawSomeCells(1000)`. |
| `BakeMaxHeights()` | Runs multiple vertical line traces per cell and stores the highest hit in `HeightCache`. |
| `GetPreviewBaseZCm() const` | Returns the base Z for the preview, either sea level or override. |
| `ClearPreviewVoxels()` | Removes the preview instances. |
| `BuildPreviewVoxels()` | Builds a visual voxel or column preview around a center point. |

---

# 10. TerrainReferenceActor.h / TerrainReferenceActor.cpp

## Name

`TerrainReferenceActor`

## Main Purpose

Manages the landscape reference and provides the world bounds of the landscape.

## Communication

- Used by `VoxelGridBaker`.

## Variables

| Variable | Type | Purpose |
|---|---|---|
| `Landscape` | `TObjectPtr<ALandscapeProxy>` | Reference to the landscape. Changes which terrain is used for bounds and grid creation. |

## Methods

| Method | Purpose |
|---|---|
| `ATerrainReferenceActor()` | Constructor. Disables tick. |
| `AutoFindLandscape()` | Automatically searches for the first `ALandscapeProxy` in the level. |
| `GetLandscapeWorldBounds(FVector& OutMin, FVector& OutMax) const` | Returns the bounding box of the landscape. |
| `IsWorldPosInsideLandscapeBounds(const FVector& WorldPos) const` | Checks whether a world position is inside the XY bounds of the landscape. |
| `DebugPrintLandscapeBounds()` | Prints bounds to the log and draws a debug box. |

---

# 11. HeightQueryProbeActor.h / HeightQueryProbeActor.cpp

## Name

`HeightQueryProbeActor`

## Main Purpose

Debug and query actor for baked height data. It can query the stored terrain height at a world position and optionally visualize the result.

## Communication

- Uses `AVoxelGridBaker` for grid origin and cell size.
- Uses `UVoxelHeightCache` for height data.
- Can be used by `VoxelGridBaker` as `PreviewCenterActor`.

## Variables

| Variable | Type | Purpose |
|---|---|---|
| `GridBaker` | `TObjectPtr<AVoxelGridBaker>` | Source for grid data. Changes which grid is queried. |
| `HeightCache` | `TObjectPtr<UVoxelHeightCache>` | Baked height data. Changes which height data is read. |
| `bDrawMarker` | `bool` | Enables the debug marker. Changes whether queries are visibly marked. |
| `MarkerLifeTime` | `float` | Lifetime of the marker. Changes how long the marker and text stay visible. |

## Methods

| Method | Purpose |
|---|---|
| `AHeightQueryProbeActor()` | Constructor. Disables tick. |
| `WorldXYToCell(const FVector& WorldPos, int32& OutX, int32& OutY) const` | Converts world XY into grid cells and checks bounds. |
| `QueryHeightAtWorldXY(const FVector& WorldPos, float& OutHeightASLm, float& OutWorldZCm) const` | Returns the terrain height at a world position in ASL meters and world Z centimeters. |
| `QueryHeightAtMyLocation()` | Queries the height at the actor location, prints the result to the log, and optionally draws a marker. |

---

# 12. FlightPathfinderActor.h / FlightPathfinderActor.cpp

## Name

`FlightPathfinderActor`

## Main Purpose

This is the main logic of the route planner. It calculates an aircraft-aware route with A*.

The search state contains X/Y/Z in the grid plus a `HeadingIndex`. This means the planner does not only plan position, but also the current flight direction.

## Communication

- Uses `VoxelGridBaker` and `VoxelHeightCache` for terrain data and search space.
- Uses `FlightProfile` for aircraft limits.
- Uses `FlightInfluenceZoneActor` for blocked zones and soft zones.
- Returns `FRouteCalculationResult` to the UI and `RoutePlannerManager`.
- Uses `ERouteFailureReason` for errors.

## Struct: FFlightPathState

| Variable | Type | Purpose |
|---|---|---|
| `X` | `int32` | Grid column. Changes the horizontal search position. |
| `Y` | `int32` | Grid row. Changes the horizontal search position. |
| `Z` | `int32` | Vertical layer. Changes the flight altitude inside the search space. |
| `HeadingIndex` | `int32` | Discrete flight direction. Changes the current movement direction. |

## Struct: FRouteFailureStats

| Variable | Type | Purpose |
|---|---|---|
| `OutOfBoundsCount` | `int32` | Counts states outside the search space. |
| `InvalidTargetStateCount` | `int32` | Counts invalid target or neighbor states. |
| `TerrainClearanceCount` | `int32` | Counts terrain-clearance failures. |
| `HardBlockZoneCount` | `int32` | Counts hits in hard blocked zones. |
| `MaxClimbExceededCount` | `int32` | Counts climbs that are too steep. |
| `MaxDescentExceededCount` | `int32` | Counts descents that are too steep. |
| `TurnRadiusTooSmallCount` | `int32` | Counts turns that are too tight. |
| `NoValidNeighborsCount` | `int32` | Counts dead ends without valid neighbors. |

## Struct: FFlightPathNodeRecord

| Variable | Type | Purpose |
|---|---|---|
| `G` | `float` | Cost from the start to this state. Lower means a better path so far. |
| `H` | `float` | Estimated remaining cost to the target. Guides A* toward the goal. |
| `F` | `float` | Priority value: `G + weighted H`. Lower values are processed earlier. |
| `Parent` | `FFlightPathState` | Previous state. Used later to reconstruct the route. |
| `bHasParent` | `bool` | Shows if a parent exists. `false` usually marks the start. |
| `bClosed` | `bool` | Shows if the state is already finished. Prevents duplicate processing. |

## Struct: FFlightOpenEntry

| Variable | Type | Purpose |
|---|---|---|
| `State` | `FFlightPathState` | Open A* candidate. |
| `FScore` | `float` | Priority inside the heap. |

## Struct: FFlightTransitionKey

| Variable | Type | Purpose |
|---|---|---|
| `From` | `FFlightPathState` | Start state of a cached movement. |
| `To` | `FFlightPathState` | Target state of a cached movement. |

## AFlightPathfinderActor Variables

| Variable | Type | Purpose |
|---|---|---|
| `GridBaker` | `TObjectPtr<AVoxelGridBaker>` | Source for grid and cache data. Changes which terrain is used. |
| `HeightCache` | `TObjectPtr<UVoxelHeightCache>` | Baked terrain heights. Changes terrain checks. |
| `FlightProfile` | `TObjectPtr<UFlightProfile>` | Active aircraft profile. Changes all flight limits. |
| `StartActor` | `TObjectPtr<AActor>` | Start point for editor search. |
| `GoalActor` | `TObjectPtr<AActor>` | Target point for editor search. |
| `InfluenceZones` | `TArray<TObjectPtr<AFlightInfluenceZoneActor>>` | Blocked or cost zones. Changes allowed or expensive airspace. |
| `VoxelSizeZMeters` | `float` | Height of one search layer. Smaller values give more accurate altitude planning but require more work. |
| `ExtraBottomLayers` | `int32` | Extra layers below the base range. Expands the search space downward. |
| `ExtraTopLayers` | `int32` | Extra layers above the base range. Expands the search space upward. |
| `HeadingBucketCount` | `int32` | Number of discrete directions. Higher values allow finer turns but create more states. |
| `MaxHeadingChangePerStep` | `int32` | Maximum direction change per step. Higher values allow stronger turns. |
| `bAllowClimbStep` | `bool` | Allows climbing steps. If `false`, active climbing is blocked. |
| `bAllowDescentStep` | `bool` | Allows descending steps. If `false`, active descending is blocked. |
| `bDrawVisitedStates` | `bool` | Draws visited search states. |
| `bAutoDrawPathAfterSearch` | `bool` | Automatically draws the route after a successful search. |
| `DebugDrawLifetime` | `float` | Lifetime of debug drawings. |
| `DebugLineThickness` | `float` | Thickness of the route line. |
| `DebugVisitedPointSize` | `float` | Size of visited debug points. |
| `FailureStats` | `FRouteFailureStats` | Debug counters for failure reasons. |
| `LastFailureReason` | `ERouteFailureReason` | Last failure reason for UI or logs. |
| `SearchMinWorld` | `FVector` | Lower world bounds of the 3D search space. |
| `SearchMaxWorld` | `FVector` | Upper world bounds of the 3D search space. |
| `ZLayerCount` | `int32` | Number of vertical search layers. |
| `HeuristicWeight` | `float` | Weight of the A* heuristic. Higher values are faster and more goal-directed, but may be less optimal. |
| `SearchMaxExpandedStates` | `int32` | Maximum number of checked A* states. Higher values allow longer searches. |
| `GoalConnectionToleranceMeters` | `float` | Distance used for direct target connection. Higher values can finish the search earlier. |
| `DirectRoutePreferredClearanceMaxLengthMeters` | `float` | Maximum distance for a direct route. `0` means no additional limit. |
| `MinimumAbsoluteTerrainClearanceMeters` | `float` | Absolute project minimum for terrain clearance. |
| `TerrainClearanceSafetyMultiplier` | `float` | Multiplier for preferred terrain clearance. |
| `LateralTerrainSafetyRadiusMultiplier` | `float` | Side safety radius for conservative terrain queries. |
| `PreferredClearanceSpeedLookaheadSeconds` | `float` | Speed-based safety buffer. |
| `MinOutputWaypointSpacingMeters` | `float` | Minimum distance after waypoint compaction. |
| `MaxOutputWaypointCompactionLookahead` | `int32` | Lookahead used during waypoint simplification. |
| `CurrentRouteWorldPoints` | `TArray<FVector>` | Current route as world points. |
| `LastExpandedStates` | `int32` | Number of states checked in the last run. |
| `PrimitiveSegmentLengthMeters` | `float` | Preferred length of one motion primitive. |
| `PrimitiveSamplesPerSegment` | `int32` | Samples per primitive. Higher values improve safety checks. |
| `PrimitiveClimbRateFactor` | `float` | Safety factor for climb and descent. Lower values are more conservative. |
| `MaxAutoPrimitiveSegmentLengthMeters` | `float` | Upper limit for automatically extended primitives. |
| `PrimitiveTurnDeltaBuckets` | `int32` | Direction change used for primitive turns. |
| `TransitionValidityCache` | `TMap<FFlightTransitionKey, bool>` | Cache for movement validation. |
| `TransitionCostCache` | `TMap<FFlightTransitionKey, float>` | Cache for movement costs. |
| `ConservativeTerrainHeightCache` | `TMap<FIntVector, float>` | Cache for conservative terrain heights. |

## FlightPathfinderActor Methods

| Method | Purpose |
|---|---|
| `AFlightPathfinderActor()` | Constructor. Disables tick. |
| `ValidateReferences() const` | Checks all references for editor search, including `StartActor` and `GoalActor`. |
| `ValidateCoreReferences() const` | Checks the core references for UI and editor search. |
| `BuildSearchSpace()` | Builds the 3D search space from terrain heights and actor positions. |
| `BuildSearchSpaceForRoute(...)` | Builds the search space for a specific UI route. |
| `GetFailureReasonText(ERouteFailureReason Reason) const` | Converts failure reasons into readable UI text. |
| `IsStateInsideBounds(const FFlightPathState& State) const` | Checks if a state is inside the grid and Z layers. |
| `StateToWorldCenter(const FFlightPathState& State) const` | Converts a search state into a world position. |
| `WorldToNearestValidState(...)` | Searches for a safe search state near a world position. |
| `WorldToStateExact(...)` | Converts a world position exactly into a search state. |
| `BuildPrimitiveSamplePoints(...)` | Creates sample points for one flight step with turn and optional climb/descent. |
| `RebuildPrimitiveSamplesBetweenStates(...)` | Rebuilds sample points for an already known transition. |
| `ApplyMotionPrimitive(...)` | Applies a motion primitive and calculates the target state. |
| `IsMotionPrimitiveValid(...)` | Checks a primitive for terrain, altitude, blocked zones, climb/descent, and turn radius. |
| `IsGoalState(...)` | Checks whether the current state matches the target in X/Y/Z. |
| `NormalizeHeadingIndex(int32 HeadingIndex) const` | Keeps the heading index inside the valid circular range. |
| `HeadingIndexToAngleRad(int32 HeadingIndex) const` | Converts a heading index into an angle in radians. |
| `HeadingIndexToGridStep(int32 HeadingIndex) const` | Converts a heading into a rough XY grid step. |
| `ComputeNearestHeadingIndexFromDirection(const FVector2D& Direction) const` | Finds the nearest heading index for a 2D direction. |
| `GetSmallestHeadingDelta(int32 FromHeading, int32 ToHeading) const` | Returns the smallest absolute heading difference. |
| `GetSignedHeadingDeltaBuckets(int32 FromHeading, int32 ToHeading) const` | Returns the shortest signed heading difference. |
| `GetTerrainHeightCmAtCell(int32 X, int32 Y) const` | Reads the baked terrain height of a cell. |
| `GetTerrainHeightCmAtWorldXY(...)` | Reads terrain height from world XY. |
| `GetConservativeTerrainHeightCmAtWorldXY(...)` | Reads the highest terrain height inside a safety radius. |
| `GetAltitudeMetersASLFromWorldZ(float WorldZCm) const` | Converts world Z into meters ASL. |
| `GetTerrainHeightMetersASLAtCell(int32 X, int32 Y) const` | Returns terrain height of a cell in ASL meters. |
| `GetRequiredTerrainClearanceMeters() const` | Determines required terrain clearance from profile and project minimum. |
| `GetPreferredTerrainClearanceMeters() const` | Calculates a larger preferred terrain clearance for cost evaluation. |
| `GetTerrainSafetyRadiusMeters() const` | Returns the radius for required terrain checks. |
| `GetPreferredTerrainSafetyRadiusMeters() const` | Returns the larger radius for preferred terrain checks. |
| `GetEffectivePrimitiveSegmentLengthMeters() const` | Calculates the actual primitive length from turn radius and climb/descent rate. |
| `GetMaxAllowedWorldZCm() const` | Converts maximum flight altitude into world Z centimeters. |
| `DoesPointRespectAltitudeLimit(const FVector& WorldPoint) const` | Checks the maximum altitude at a point. |
| `DoesRouteRespectAltitudeLimit(const TArray<FVector>& RoutePoints) const` | Checks the maximum altitude for all route points. |
| `DoesSegmentRespectAltitudeLimit(...)` | Checks maximum altitude along a segment. |
| `DoesPointRespectTerrainClearance(const FVector& WorldPoint) const` | Checks required terrain clearance at a point. |
| `DoesPointRespectPreferredTerrainClearance(const FVector& WorldPoint) const` | Checks preferred terrain clearance at a point. |
| `DoesSegmentRespectPreferredTerrainClearance(...)` | Checks preferred terrain clearance along a segment. |
| `DoesDirectSegmentRespectFlightRules(...)` | Checks a direct segment against all main flight rules. |
| `TryBuildDirectVfrRoute(...)` | Tries a safe two-point route before A* starts. |
| `CanConnectToGoal(...)` | Checks if a search state can safely connect directly to the target. |
| `DoesRouteRespectTurnRadius(...)` | Checks whether waypoint turns respect the minimum turn radius. |
| `ValidateCurrentRouteSafety() const` | Runs the final safety check for the current route. |
| `CompactCurrentRouteWaypoints()` | Removes unnecessary intermediate points while keeping the route safe. |
| `IsStateValid(const FFlightPathState& State) const` | Checks if a search state is safe and usable. |
| `DoesSegmentRespectTerrainClearance(...)` | Checks required terrain clearance along a segment. |
| `IsTransitionValid(const FFlightPathState& From, const FFlightPathState& To) const` | Checks if movement between two states is allowed. |
| `IsStateInsideHardBlockZone(...)` | Checks if a point is inside a hard blocked zone. |
| `DoesSegmentIntersectHardBlockZone(...)` | Checks if a segment intersects a hard blocked zone. |
| `GetSoftZoneTraversalCost(...)` | Calculates additional cost for soft zones. |
| `GetNeighbors(...)` | Generates valid neighbor states from heading and vertical options. |
| `HeuristicCost(...)` | Estimates remaining cost to the target for A*. |
| `TransitionCost(...)` | Calculates movement cost using flight time, zones, climb/descent, turns, and terrain distance. |
| `IsTransitionValidCached(...)` | Uses a cache for movement validation. |
| `TransitionCostCached(...)` | Uses a cache for movement costs. |
| `CalculateTurnReversalPenalty(...)` | Penalizes direct left-right or right-left jitter. |
| `PopBestOpenStateFromHeap(...)` | Gets the best open A* state from the heap and ignores outdated entries. |
| `ReconstructRoute(...)` | Builds the route from parent links in the A* records. |
| `CalculateRouteLengthMeters() const` | Calculates the total length of the current route. |
| `CalculateNetAltitudeChangeMeters() const` | Calculates the altitude difference between start and target. |
| `CalculateTotalClimbMeters() const` | Sums all positive climb sections. |
| `RunFlightRouteSearch(...)` | Complete internal search flow: reset, direct route attempt, A*, reconstruction, compaction, and safety check. |
| `CalculateFlightRouteForUI(...)` | UI method. Checks altitudes, converts ASL to world Z, and returns `FRouteCalculationResult`. |
| `FindFlightRoute()` | Editor method for `StartActor` and `GoalActor`. |
| `DebugDrawCurrentRoute()` | Draws the route with lines and spheres. |
| `ClearCurrentRoute()` | Clears the route and debug drawings. |

## Additional Struct and Helper Functions

| Function | Purpose |
|---|---|
| `FFlightPathState()` | Default constructor. |
| `FFlightPathState(int32 InX, int32 InY, int32 InZ, int32 InHeadingIndex)` | Constructor for a search state. |
| `FFlightPathState::operator==(...)` | Compares X, Y, Z, and `HeadingIndex`. |
| `FRouteFailureStats::Reset()` | Resets all failure counters. |
| `GetTypeHash(const FFlightPathState& State)` | Hash function for `TMap` and `TSet`. |
| `FFlightOpenEntry()` | Default constructor. |
| `FFlightOpenEntry(const FFlightPathState& InState, float InFScore)` | Constructor for heap entries. |
| `FFlightOpenEntryMinHeapPredicate::operator()(...)` | Sorts the heap by `FScore`. |
| `FFlightTransitionKey()` | Default constructor. |
| `FFlightTransitionKey(const FFlightPathState& InFrom, const FFlightPathState& InTo)` | Constructor for cache keys. |
| `FFlightTransitionKey::operator==(...)` | Compares `From` and `To`. |
| `GetTypeHash(const FFlightTransitionKey& Key)` | Hash function for transition caches. |

---

# 13. Simple3DPathfinderActor.h / Simple3DPathfinderActor.cpp

## Name

`Simple3DPathfinderActor`

## Main Purpose

A simpler 3D A* pathfinder based on voxels. It works like a prototype or a simpler alternative to `FlightPathfinderActor`.

It respects terrain blocking, can prevent corner cutting, and can smooth paths. However, it does not plan flight direction with motion primitives.

## Communication

- Uses `VoxelGridBaker` and `VoxelHeightCache`.
- Uses `FlightProfile` for basic validation.
- Uses `StartActor` and `GoalActor` as endpoints.

## Struct: FGridVoxelCoord

| Variable | Type | Purpose |
|---|---|---|
| `X` | `int32` | Grid column. Changes horizontal position. |
| `Y` | `int32` | Grid row. Changes horizontal position. |
| `Z` | `int32` | Vertical layer. Changes altitude in voxel space. |

## Struct: FGridVoxelNodeRecord

| Variable | Type | Purpose |
|---|---|---|
| `G` | `float` | Cost from the start to this voxel. |
| `H` | `float` | Estimated remaining cost to the target. |
| `F` | `float` | Priority value: `G + H`. |
| `Parent` | `FGridVoxelCoord` | Previous voxel for path reconstruction. |
| `bHasParent` | `bool` | Shows if a parent exists. |
| `bClosed` | `bool` | Shows if the node is already finished. |

## ASimple3DPathfinderActor Variables

| Variable | Type | Purpose |
|---|---|---|
| `GridBaker` | `TObjectPtr<AVoxelGridBaker>` | Source for grid data. |
| `HeightCache` | `TObjectPtr<UVoxelHeightCache>` | Baked terrain heights. |
| `StartActor` | `TObjectPtr<AActor>` | Start point. |
| `GoalActor` | `TObjectPtr<AActor>` | Target point. |
| `VoxelSizeZMeters` | `float` | Height of one voxel layer. |
| `ExtraBottomLayers` | `int32` | Extra layers below the base range. |
| `ExtraTopLayers` | `int32` | Extra layers above the base range. |
| `bDrawVisitedNodes` | `bool` | Draws visited A* nodes. |
| `DebugDrawLifetime` | `float` | Lifetime of debug drawings. |
| `DebugLineThickness` | `float` | Thickness of the path line. |
| `DebugVisitedNodeSize` | `float` | Size of visited debug points. |
| `VoxelMinWorld` | `FVector` | Lower world bounds of the voxel space. |
| `VoxelMaxWorld` | `FVector` | Upper world bounds of the voxel space. |
| `ZLayerCount` | `int32` | Number of vertical voxel layers. |
| `CurrentPathWorldPoints` | `TArray<FVector>` | Current path as world points. |
| `bPreventCornerCutting` | `bool` | Prevents diagonal movement through blocked corners. |
| `bEnablePathSmoothing` | `bool` | Enables path smoothing after A*. |
| `MaxSmoothSkipPoints` | `int32` | Maximum number of skipped points during smoothing. |
| `FlightProfile` | `TObjectPtr<UFlightProfile>` | Flight profile used for basic validation. |

## Methods

| Method | Purpose |
|---|---|
| `ASimple3DPathfinderActor()` | Constructor. Disables tick. |
| `ValidateReferences() const` | Checks `GridBaker`, `HeightCache`, `StartActor`, `GoalActor`, and `FlightProfile`. |
| `BuildVoxelSpace()` | Builds the 3D voxel search space from terrain heights and start/target heights. |
| `WorldToVoxel(...)` | Converts a world position exactly into a voxel coordinate. |
| `WorldToNearestFreeVoxel(...)` | Searches for the nearest non-blocked voxel at the same XY position. |
| `VoxelToWorldCenter(...)` | Converts a voxel coordinate into the world center position. |
| `IsVoxelInsideBounds(...)` | Checks if a voxel is inside the search space. |
| `GetTerrainHeightCmAtCell(...)` | Reads terrain height of a cell. |
| `GetTerrainHeightCmAtWorldXY(...)` | Reads terrain height from world XY. |
| `IsVoxelBlocked(...)` | Checks if a voxel is outside, invalid, or inside terrain. |
| `IsMoveAllowed(...)` | Checks if movement to a neighbor voxel is allowed and optionally prevents corner cutting. |
| `HeuristicCost(...)` | Estimates remaining cost to the target as world distance. |
| `MovementCost(...)` | Calculates movement cost as world distance. |
| `GetNeighbors(...)` | Collects allowed neighbor voxels in a 26-neighbor area. |
| `FindBestOpenNode(...)` | Finds the open node with the lowest F value. |
| `CanTravelDirect(...)` | Checks if a direct line between two points stays above terrain. |
| `SmoothPath()` | Removes intermediate points when later points can be reached directly and safely. |
| `ReconstructPath(...)` | Builds the world path from parent links. |
| `FindPath()` | Runs the simple A* search, reconstructs/smooths the path, and draws it. |
| `DebugDrawCurrentPath()` | Draws the path with lines and spheres. |
| `ClearCurrentPath()` | Clears the path and debug drawings. |

## Additional Struct and Helper Functions

| Function | Purpose |
|---|---|
| `FGridVoxelCoord()` | Default constructor. |
| `FGridVoxelCoord(int32 InX, int32 InY, int32 InZ)` | Constructor for coordinates. |
| `FGridVoxelCoord::operator==(...)` | Compares X, Y, and Z. |
| `GetTypeHash(const FGridVoxelCoord& Coord)` | Hash function for `TMap` and `TSet`. |

---

# Summary of the System Collaboration

1. **Prepare terrain:** `TerrainReferenceActor` provides the landscape bounds. `VoxelGridBaker` uses `VoxelGridConfig` to build the grid and stores height values in `VoxelHeightCache`.
2. **Request route:** The UI sets markers, altitudes, and the aircraft profile in `RoutePlannerManager`.
3. **Calculate route:** The manager calls `CalculateFlightRouteForUI()` in `FlightPathfinderActor`.
4. **Convert values:** UI altitudes in meters ASL are converted to world Z using `SeaLevelWorldZCm`.
5. **Check safety:** Start, target, route segments, terrain clearance, flight altitude, restricted zones, climb/descent, and turn radius are validated.
6. **Search route:** If no direct route is safe, A* runs over `FFlightPathState`, which stores both position and direction.
7. **Return result:** The route is reconstructed, simplified, checked one final time, and returned to the UI as `FRouteCalculationResult`.
