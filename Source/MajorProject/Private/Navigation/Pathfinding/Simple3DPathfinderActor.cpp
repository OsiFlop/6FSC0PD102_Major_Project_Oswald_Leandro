// Simple 3D pathfinder actor
// Builds voxel search space from baked terrain data
// Converts world positions into voxel coordinates
// Runs A* pathfinding between start and goal actor
// Prevents terrain collision and optional corner cutting
// Supports path smoothing and debug drawing

#include "Navigation/Pathfinding/Simple3DPathfinderActor.h"
#include "Navigation/Grid/VoxelGridBaker.h"
#include "Navigation/Grid/VoxelHeightCache.h"
#include "FlightProfile.h"
#include "DrawDebugHelpers.h"

// Sets default values
ASimple3DPathfinderActor::ASimple3DPathfinderActor()
{
	PrimaryActorTick.bCanEverTick = false;
}

// Check all required references and flight profile values
bool ASimple3DPathfinderActor::ValidateReferences() const
{
	if (!GridBaker)
	{
		UE_LOG(LogTemp, Warning, TEXT("Pathfinder: GridBaker fehlt."));
		return false;
	}

	if (!HeightCache)
	{
		UE_LOG(LogTemp, Warning, TEXT("Pathfinder: HeightCache fehlt."));
		return false;
	}

	if (!HeightCache->IsValid())
	{
		UE_LOG(LogTemp, Warning, TEXT("Pathfinder: HeightCache ist ungueltig. Erst Grid backen."));
		return false;
	}

	if (!StartActor || !GoalActor)
	{
		UE_LOG(LogTemp, Warning, TEXT("Pathfinder: StartActor oder GoalActor fehlt."));
		return false;
	}
	
	if (!FlightProfile)
	{
		UE_LOG(LogTemp, Warning, TEXT("Pathfinder: FlightProfile fehlt."));
		return false;
	}

	// CruiseSpeedMetersPerSecond: needed for valid aircraft movement data
	if (FlightProfile->CruiseSpeedMetersPerSecond <= 0.0f)
	{
		UE_LOG(LogTemp, Warning, TEXT("Pathfinder: CruiseSpeedMetersPerSecond muss > 0 sein."));
		return false;
	}
	
	return true;
}

// Build vertical voxel space from terrain height range
void ASimple3DPathfinderActor::BuildVoxelSpace()
{
	if (!GridBaker)
	{
		UE_LOG(LogTemp, Warning, TEXT("BuildVoxelSpace fehlgeschlagen: GridBaker fehlt."));
		return;
	}

	// Use cache from baker if no cache assigned
	if (!HeightCache && GridBaker->HeightCache)
	{
		HeightCache = GridBaker->HeightCache;
	}

	if (!HeightCache || !HeightCache->IsValid())
	{
		UE_LOG(LogTemp, Warning, TEXT("BuildVoxelSpace fehlgeschlagen: HeightCache fehlt oder ist ungueltig."));
		return;
	}

	if (VoxelSizeZMeters <= 0.0f)
	{
		UE_LOG(LogTemp, Warning, TEXT("BuildVoxelSpace fehlgeschlagen: VoxelSizeZMeters muss > 0 sein."));
		return;
	}

	float MinTerrainZ = TNumericLimits<float>::Max();
	float MaxTerrainZ = -TNumericLimits<float>::Max();

	// MinTerrainZ / MaxTerrainZ: terrain height range from cache
	for (const float HeightCm : HeightCache->MaxHeightCm)
	{
		if (HeightCm <= -1e20f)
		{
			continue;
		}

		MinTerrainZ = FMath::Min(MinTerrainZ, HeightCm);
		MaxTerrainZ = FMath::Max(MaxTerrainZ, HeightCm);
	}

	if (MinTerrainZ == TNumericLimits<float>::Max() || MaxTerrainZ == -TNumericLimits<float>::Max())
	{
		UE_LOG(LogTemp, Warning, TEXT("BuildVoxelSpace fehlgeschlagen: Keine gueltigen Terrainhoehen im Cache."));
		return;
	}

	// VoxelSizeZCm: vertical voxel size in Unreal centimeters
	const float VoxelSizeZCm = VoxelSizeZMeters * 100.0f;

	float MinRelevantZ = MinTerrainZ;
	float MaxRelevantZ = MaxTerrainZ;

	// Include start actor height in voxel space
	if (StartActor)
	{
		MinRelevantZ = FMath::Min(MinRelevantZ, StartActor->GetActorLocation().Z);
		MaxRelevantZ = FMath::Max(MaxRelevantZ, StartActor->GetActorLocation().Z);
	}

	// Include goal actor height in voxel space
	if (GoalActor)
	{
		MinRelevantZ = FMath::Min(MinRelevantZ, GoalActor->GetActorLocation().Z);
		MaxRelevantZ = FMath::Max(MaxRelevantZ, GoalActor->GetActorLocation().Z);
	}

	// VoxelMinWorld / VoxelMaxWorld: full 3D search bounds
	VoxelMinWorld.X = HeightCache->GridMinWorld.X;
	VoxelMinWorld.Y = HeightCache->GridMinWorld.Y;
	VoxelMinWorld.Z = MinRelevantZ - (ExtraBottomLayers * VoxelSizeZCm);

	VoxelMaxWorld.X = HeightCache->GridMinWorld.X + (HeightCache->GridSize.X * HeightCache->CellSizeCm);
	VoxelMaxWorld.Y = HeightCache->GridMinWorld.Y + (HeightCache->GridSize.Y * HeightCache->CellSizeCm);
	VoxelMaxWorld.Z = MaxRelevantZ + (ExtraTopLayers * VoxelSizeZCm);

	// ZLayerCount: amount of vertical layers for A*
	const float HeightRangeZ = VoxelMaxWorld.Z - VoxelMinWorld.Z;
	ZLayerCount = FMath::Max(1, FMath::CeilToInt(HeightRangeZ / VoxelSizeZCm));

	UE_LOG(LogTemp, Display, TEXT("VoxelSpace gebaut: XY=%d x %d, ZLayers=%d, VoxelSizeZ=%.2fm"),
	       HeightCache->GridSize.X,
	       HeightCache->GridSize.Y,
	       ZLayerCount,
	       VoxelSizeZMeters);
}

// Convert world position to exact voxel coordinate
bool ASimple3DPathfinderActor::WorldToVoxel(const FVector& WorldPos, FGridVoxelCoord& OutVoxel) const
{
	if (!HeightCache || !HeightCache->IsValid())
	{
		return false;
	}

	if (ZLayerCount <= 0 || VoxelSizeZMeters <= 0.0f)
	{
		return false;
	}

	// LocalX / LocalY: position relative to grid origin
	const float LocalX = WorldPos.X - HeightCache->GridMinWorld.X;
	const float LocalY = WorldPos.Y - HeightCache->GridMinWorld.Y;

	const int32 GridX = FMath::FloorToInt(LocalX / HeightCache->CellSizeCm);
	const int32 GridY = FMath::FloorToInt(LocalY / HeightCache->CellSizeCm);

	// Reject positions outside baked XY grid
	if (GridX < 0 || GridY < 0 || GridX >= HeightCache->GridSize.X || GridY >= HeightCache->GridSize.Y)
	{
		return false;
	}

	const float VoxelSizeZCm = VoxelSizeZMeters * 100.0f;
	const float LocalZ = WorldPos.Z - VoxelMinWorld.Z;
	const int32 GridZ = FMath::FloorToInt(LocalZ / VoxelSizeZCm);

	// Reject positions outside vertical voxel space
	if (GridZ < 0 || GridZ >= ZLayerCount)
	{
		return false;
	}

	OutVoxel = FGridVoxelCoord(GridX, GridY, GridZ);
	return true;
}

// Find nearest free voxel at same XY position
bool ASimple3DPathfinderActor::WorldToNearestFreeVoxel(const FVector& WorldPos, FGridVoxelCoord& OutVoxel) const
{
	if (!HeightCache || !HeightCache->IsValid())
	{
		return false;
	}

	const float LocalX = WorldPos.X - HeightCache->GridMinWorld.X;
	const float LocalY = WorldPos.Y - HeightCache->GridMinWorld.Y;

	const int32 GridX = FMath::FloorToInt(LocalX / HeightCache->CellSizeCm);
	const int32 GridY = FMath::FloorToInt(LocalY / HeightCache->CellSizeCm);

	if (GridX < 0 || GridY < 0 || GridX >= HeightCache->GridSize.X || GridY >= HeightCache->GridSize.Y)
	{
		return false;
	}

	const float VoxelSizeZCm = VoxelSizeZMeters * 100.0f;

	// StartZ: closest vertical layer to actor height
	int32 StartZ = FMath::FloorToInt((WorldPos.Z - VoxelMinWorld.Z) / VoxelSizeZCm);
	StartZ = FMath::Clamp(StartZ, 0, ZLayerCount - 1);

	// Prefer free voxel above start height
	for (int32 Z = StartZ; Z < ZLayerCount; ++Z)
	{
		const FGridVoxelCoord Candidate(GridX, GridY, Z);
		if (!IsVoxelBlocked(Candidate))
		{
			OutVoxel = Candidate;
			return true;
		}
	}

	// Fallback: search below start height
	for (int32 Z = StartZ - 1; Z >= 0; --Z)
	{
		const FGridVoxelCoord Candidate(GridX, GridY, Z);
		if (!IsVoxelBlocked(Candidate))
		{
			OutVoxel = Candidate;
			return true;
		}
	}

	return false;
}

// Convert voxel coordinate to world-space center point
FVector ASimple3DPathfinderActor::VoxelToWorldCenter(const FGridVoxelCoord& Voxel) const
{
	const float WorldX = HeightCache->GridMinWorld.X + (static_cast<float>(Voxel.X) + 0.5f) * HeightCache->CellSizeCm;
	const float WorldY = HeightCache->GridMinWorld.Y + (static_cast<float>(Voxel.Y) + 0.5f) * HeightCache->CellSizeCm;

	const float VoxelSizeZCm = VoxelSizeZMeters * 100.0f;
	const float WorldZ = VoxelMinWorld.Z + (static_cast<float>(Voxel.Z) + 0.5f) * VoxelSizeZCm;

	return FVector(WorldX, WorldY, WorldZ);
}

// Check if voxel coordinate is inside search bounds
bool ASimple3DPathfinderActor::IsVoxelInsideBounds(const FGridVoxelCoord& Voxel) const
{
	if (!HeightCache)
	{
		return false;
	}

	return
		Voxel.X >= 0 && Voxel.X < HeightCache->GridSize.X &&
		Voxel.Y >= 0 && Voxel.Y < HeightCache->GridSize.Y &&
		Voxel.Z >= 0 && Voxel.Z < ZLayerCount;
}

// Get terrain height from cache cell
float ASimple3DPathfinderActor::GetTerrainHeightCmAtCell(const int32 X, const int32 Y) const
{
	if (!HeightCache || !HeightCache->IsValid())
	{
		return -FLT_MAX;
	}

	const int32 Index = HeightCache->ToIndex(X, Y);
	if (!HeightCache->MaxHeightCm.IsValidIndex(Index))
	{
		return -FLT_MAX;
	}

	return HeightCache->MaxHeightCm[Index];
}

// Get terrain height from world X / Y position
bool ASimple3DPathfinderActor::GetTerrainHeightCmAtWorldXY(float WorldX, float WorldY, float& OutTerrainHeightCm) const
{
	if (!HeightCache || !HeightCache->IsValid())
	{
		return false;
	}

	// Convert world XY into grid cell
	const float LocalX = WorldX - HeightCache->GridMinWorld.X;
	const float LocalY = WorldY - HeightCache->GridMinWorld.Y;

	const int32 GridX = FMath::FloorToInt(LocalX / HeightCache->CellSizeCm);
	const int32 GridY = FMath::FloorToInt(LocalY / HeightCache->CellSizeCm);

	if (GridX < 0 || GridY < 0 || GridX >= HeightCache->GridSize.X || GridY >= HeightCache->GridSize.Y)
	{
		return false;
	}

	const float TerrainHeightCm = GetTerrainHeightCmAtCell(GridX, GridY);
	if (TerrainHeightCm <= -1e20f)
	{
		return false;
	}

	OutTerrainHeightCm = TerrainHeightCm;
	return true;
}

// Check if voxel is blocked by terrain or invalid data
bool ASimple3DPathfinderActor::IsVoxelBlocked(const FGridVoxelCoord& Voxel) const
{
	if (!IsVoxelInsideBounds(Voxel))
	{
		return true;
	}

	const float TerrainHeightCm = GetTerrainHeightCmAtCell(Voxel.X, Voxel.Y);
	if (TerrainHeightCm <= -1e20f)
	{
		return true;
	}

	const FVector VoxelCenter = VoxelToWorldCenter(Voxel);

	// Main rule: voxel center must be above terrain
	if (VoxelCenter.Z <= TerrainHeightCm)
	{
		return true;
	}

	return false;
}

// Check if move between two neighboring voxels is allowed
bool ASimple3DPathfinderActor::IsMoveAllowed(const FGridVoxelCoord& From, const FGridVoxelCoord& To) const
{
	if (!IsVoxelInsideBounds(To) || IsVoxelBlocked(To))
	{
		return false;
	}

	if (!bPreventCornerCutting)
	{
		return true;
	}

	// DX / DY / DZ: movement direction per axis
	const int32 DX = To.X - From.X;
	const int32 DY = To.Y - From.Y;
	const int32 DZ = To.Z - From.Z;

	// ChangedAxes: diagonal move if value is greater than 1
	const int32 ChangedAxes =
		(DX != 0 ? 1 : 0) +
		(DY != 0 ? 1 : 0) +
		(DZ != 0 ? 1 : 0);

	if (ChangedAxes <= 1)
	{
		return true;
	}

	// Block diagonal move if side voxel in X direction is blocked
	if (DX != 0)
	{
		const FGridVoxelCoord StepX(From.X + DX, From.Y, From.Z);
		if (IsVoxelBlocked(StepX))
		{
			return false;
		}
	}

	// Block diagonal move if side voxel in Y direction is blocked
	if (DY != 0)
	{
		const FGridVoxelCoord StepY(From.X, From.Y + DY, From.Z);
		if (IsVoxelBlocked(StepY))
		{
			return false;
		}
	}

	// Block diagonal move if side voxel in Z direction is blocked
	if (DZ != 0)
	{
		const FGridVoxelCoord StepZ(From.X, From.Y, From.Z + DZ);
		if (IsVoxelBlocked(StepZ))
		{
			return false;
		}
	}

	return true;
}

// Estimate remaining A* cost to goal
float ASimple3DPathfinderActor::HeuristicCost(const FGridVoxelCoord& A, const FGridVoxelCoord& B) const
{
	return FVector::Distance(VoxelToWorldCenter(A), VoxelToWorldCenter(B));
}

// Calculate movement cost between two voxels
float ASimple3DPathfinderActor::MovementCost(const FGridVoxelCoord& A, const FGridVoxelCoord& B) const
{
	return FVector::Distance(VoxelToWorldCenter(A), VoxelToWorldCenter(B));
}

// Collect allowed neighbor voxels around center
void ASimple3DPathfinderActor::GetNeighbors(const FGridVoxelCoord& Center, TArray<FGridVoxelCoord>& OutNeighbors) const
{
	OutNeighbors.Reset();

	// 26-neighbor search, including diagonals
	for (int32 DZ = -1; DZ <= 1; ++DZ)
	{
		for (int32 DY = -1; DY <= 1; ++DY)
		{
			for (int32 DX = -1; DX <= 1; ++DX)
			{
				if (DX == 0 && DY == 0 && DZ == 0)
				{
					continue;
				}

				const FGridVoxelCoord Neighbor(Center.X + DX, Center.Y + DY, Center.Z + DZ);

				if (!IsVoxelInsideBounds(Neighbor))
				{
					continue;
				}

				if (IsMoveAllowed(Center, Neighbor))
				{
					OutNeighbors.Add(Neighbor);
				}
			}
		}
	}
}

// Find open A* node with lowest F cost
bool ASimple3DPathfinderActor::FindBestOpenNode(
	const TSet<FGridVoxelCoord>& OpenSet,
	const TMap<FGridVoxelCoord, FGridVoxelNodeRecord>& Records,
	FGridVoxelCoord& OutBestNode
) const
{
	float BestF = TNumericLimits<float>::Max();
	bool bFound = false;

	// OpenSet: all nodes waiting for A* check
	for (const FGridVoxelCoord& Candidate : OpenSet)
	{
		const FGridVoxelNodeRecord* Record = Records.Find(Candidate);
		if (!Record || Record->bClosed)
		{
			continue;
		}

		if (Record->F < BestF)
		{
			BestF = Record->F;
			OutBestNode = Candidate;
			bFound = true;
		}
	}

	return bFound;
}

// Check if straight segment between two path points is terrain-free
bool ASimple3DPathfinderActor::CanTravelDirect(const FVector& From, const FVector& To) const
{
	const float Distance = FVector::Distance(From, To);
	if (Distance <= KINDA_SMALL_NUMBER)
	{
		return true;
	}

	// StepSize: half cell size for safer terrain sampling
	const float StepSize = HeightCache->CellSizeCm * 0.5f;
	const int32 NumSteps = FMath::Max(1, FMath::CeilToInt(Distance / StepSize));

	for (int32 Step = 0; Step <= NumSteps; ++Step)
	{
		const float Alpha = static_cast<float>(Step) / static_cast<float>(NumSteps);
		const FVector Sample = FMath::Lerp(From, To, Alpha);

		float TerrainHeightCm = 0.0f;
		if (!GetTerrainHeightCmAtWorldXY(Sample.X, Sample.Y, TerrainHeightCm))
		{
			return false;
		}

		// Direct line must stay above terrain
		if (Sample.Z <= TerrainHeightCm)
		{
			return false;
		}
	}

	return true;
}

// Simplify path by skipping unnecessary intermediate points
void ASimple3DPathfinderActor::SmoothPath()
{
	if (CurrentPathWorldPoints.Num() < 3)
	{
		return;
	}

	TArray<FVector> Smoothed;
	
	// CurrentIndex: last accepted point in smoothed path
	int32 CurrentIndex = 0;
	Smoothed.Add(CurrentPathWorldPoints[0]);

	while (CurrentIndex < CurrentPathWorldPoints.Num() - 1)
	{
		// FurthestReachable: best next point with direct travel
		int32 FurthestReachable = CurrentIndex + 1;

		const int32 MaxTestIndex = FMath::Min(
			CurrentPathWorldPoints.Num() - 1,
			CurrentIndex + MaxSmoothSkipPoints
		);

		// Test farthest point first
		for (int32 TestIndex = MaxTestIndex; TestIndex > CurrentIndex; --TestIndex)
		{
			if (CanTravelDirect(CurrentPathWorldPoints[CurrentIndex], CurrentPathWorldPoints[TestIndex]))
			{
				FurthestReachable = TestIndex;
				break;
			}
		}

		Smoothed.Add(CurrentPathWorldPoints[FurthestReachable]);
		CurrentIndex = FurthestReachable;
	}

	CurrentPathWorldPoints = Smoothed;
}

// Rebuild world-space path from A* parent records
void ASimple3DPathfinderActor::ReconstructPath(
	const TMap<FGridVoxelCoord, FGridVoxelNodeRecord>& Records,
	const FGridVoxelCoord& GoalVoxel
)
{
	CurrentPathWorldPoints.Reset();

	FGridVoxelCoord Current = GoalVoxel;

	// Walk backwards from goal to start
	while (true)
	{
		CurrentPathWorldPoints.Insert(VoxelToWorldCenter(Current), 0);

		const FGridVoxelNodeRecord* Record = Records.Find(Current);
		if (!Record || !Record->bHasParent)
		{
			break;
		}

		Current = Record->Parent;
	}
}

// Run A* pathfinding from start actor to goal actor
void ASimple3DPathfinderActor::FindPath()
{
	CurrentPathWorldPoints.Reset();

	// Auto-link height cache from baker
	if (!HeightCache && GridBaker && GridBaker->HeightCache)
	{
		HeightCache = GridBaker->HeightCache;
	}

	if (!ValidateReferences())
	{
		return;
	}

	if (ZLayerCount <= 0)
	{
		BuildVoxelSpace();
	}

	if (ZLayerCount <= 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("FindPath abgebrochen: VoxelSpace konnte nicht aufgebaut werden."));
		return;
	}

	FGridVoxelCoord StartVoxel;
	FGridVoxelCoord GoalVoxel;

	if (!WorldToNearestFreeVoxel(StartActor->GetActorLocation(), StartVoxel))
	{
		UE_LOG(LogTemp, Warning, TEXT("FindPath abgebrochen: Kein freier Start Voxel gefunden."));
		return;
	}

	if (!WorldToNearestFreeVoxel(GoalActor->GetActorLocation(), GoalVoxel))
	{
		UE_LOG(LogTemp, Warning, TEXT("FindPath abgebrochen: Kein freier Ziel Voxel gefunden."));
		return;
	}

	// OpenSet: nodes waiting for A* evaluation
	TSet<FGridVoxelCoord> OpenSet;

	// Records: cost, parent and closed state for each visited voxel
	TMap<FGridVoxelCoord, FGridVoxelNodeRecord> Records;

	// StartRecord: first A* node
	FGridVoxelNodeRecord StartRecord;
	StartRecord.G = 0.0f;
	StartRecord.H = HeuristicCost(StartVoxel, GoalVoxel);
	StartRecord.F = StartRecord.G + StartRecord.H;
	StartRecord.bHasParent = false;
	StartRecord.bClosed = false;

	Records.Add(StartVoxel, StartRecord);
	OpenSet.Add(StartVoxel);

	int32 IterationCount = 0;
	const int32 MaxIterations = 200000;

	bool bPathFound = false;

	while (OpenSet.Num() > 0 && IterationCount < MaxIterations)
	{
		++IterationCount;

		FGridVoxelCoord Current;
		if (!FindBestOpenNode(OpenSet, Records, Current))
		{
			break;
		}

		OpenSet.Remove(Current);

		FGridVoxelNodeRecord* CurrentRecord = Records.Find(Current);
		if (!CurrentRecord)
		{
			continue;
		}

		if (CurrentRecord->bClosed)
		{
			continue;
		}

		// Mark node as fully checked
		CurrentRecord->bClosed = true;

		// Optional debug for visited nodes
		if (bDrawVisitedNodes && GetWorld())
		{
			DrawDebugPoint(
				GetWorld(),
				VoxelToWorldCenter(Current),
				DebugVisitedNodeSize,
				FColor::Blue,
				false,
				DebugDrawLifetime
			);
		}

		if (Current == GoalVoxel)
		{
			bPathFound = true;
			break;
		}

		TArray<FGridVoxelCoord> Neighbors;
		GetNeighbors(Current, Neighbors);

		for (const FGridVoxelCoord& Neighbor : Neighbors)
		{
			FGridVoxelNodeRecord* ExistingNeighborRecord = Records.Find(Neighbor);
			if (ExistingNeighborRecord && ExistingNeighborRecord->bClosed)
			{
				continue;
			}

			// TentativeG: new possible cost from start to neighbor
			const float TentativeG = CurrentRecord->G + MovementCost(Current, Neighbor);

			bool bShouldUpdate = false;

			// First time reaching this neighbor
			if (!ExistingNeighborRecord)
			{
				FGridVoxelNodeRecord NewRecord;
				Records.Add(Neighbor, NewRecord);
				ExistingNeighborRecord = Records.Find(Neighbor);
				bShouldUpdate = true;
			}
			else if (TentativeG < ExistingNeighborRecord->G)
			{
				bShouldUpdate = true;
			}

			if (!bShouldUpdate || !ExistingNeighborRecord)
			{
				continue;
			}

			// Store better path data for neighbor
			ExistingNeighborRecord->G = TentativeG;
			ExistingNeighborRecord->H = HeuristicCost(Neighbor, GoalVoxel);
			ExistingNeighborRecord->F = ExistingNeighborRecord->G + ExistingNeighborRecord->H;
			ExistingNeighborRecord->Parent = Current;
			ExistingNeighborRecord->bHasParent = true;
			ExistingNeighborRecord->bClosed = false;

			OpenSet.Add(Neighbor);
		}
	}

	if (!bPathFound)
	{
		UE_LOG(LogTemp, Warning, TEXT("FindPath: Kein Pfad gefunden. Iterationen=%d"), IterationCount);
		return;
	}

	ReconstructPath(Records, GoalVoxel);

	if (bEnablePathSmoothing)
	{
		SmoothPath();
	}

	UE_LOG(LogTemp, Display, TEXT("FindPath erfolgreich. Punkte im Pfad=%d, Iterationen=%d"),
	       CurrentPathWorldPoints.Num(),
	       IterationCount);

	DebugDrawCurrentPath();
}

// Draw current path with persistent debug lines and spheres
void ASimple3DPathfinderActor::DebugDrawCurrentPath()
{
	if (!GetWorld())
	{
		return;
	}

	if (CurrentPathWorldPoints.Num() < 2)
	{
		UE_LOG(LogTemp, Warning, TEXT("DebugDrawCurrentPath: Kein gueltiger Pfad vorhanden."));
		return;
	}

	for (int32 i = 0; i < CurrentPathWorldPoints.Num() - 1; ++i)
	{
		DrawDebugLine(
			GetWorld(),
			CurrentPathWorldPoints[i],
			CurrentPathWorldPoints[i + 1],
			FColor::Red,
			true,
			-1.0f,
			0,
			DebugLineThickness
		);

		DrawDebugSphere(
			GetWorld(),
			CurrentPathWorldPoints[i],
			40.0f,
			8,
			FColor::Yellow,
			true,
			-1.0f,
			0,
			2.0f
		);
	}

	DrawDebugSphere(
		GetWorld(),
		CurrentPathWorldPoints.Last(),
		40.0f,
		8,
		FColor::Yellow,
		true,
		-1.0f,
		0,
		2.0f
	);
}

// Clear path result and persistent debug drawings
void ASimple3DPathfinderActor::ClearCurrentPath()
{
	CurrentPathWorldPoints.Reset();

	if (GetWorld())
	{
		FlushPersistentDebugLines(GetWorld());
		FlushDebugStrings(GetWorld());
	}

	UE_LOG(LogTemp, Display, TEXT("Aktueller Pfad wurde geleert."));
}