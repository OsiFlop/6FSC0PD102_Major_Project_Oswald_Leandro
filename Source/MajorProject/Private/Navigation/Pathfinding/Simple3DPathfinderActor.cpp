// Fill out your copyright notice in the Description page of Project Settings.


#include "Navigation/Pathfinding/Simple3DPathfinderActor.h"
#include "Navigation/Grid/VoxelGridBaker.h"
#include "Navigation/Grid/VoxelHeightCache.h"
#include "FlightProfile.h"
#include "DrawDebugHelpers.h"

// Sets default values
ASimple3DPathfinderActor::ASimple3DPathfinderActor()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = false;
}

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

	if (FlightProfile->CruiseSpeedMetersPerSecond <= 0.0f)
	{
		UE_LOG(LogTemp, Warning, TEXT("Pathfinder: CruiseSpeedMetersPerSecond muss > 0 sein."));
		return false;
	}


	return true;
}

void ASimple3DPathfinderActor::BuildVoxelSpace()
{
	if (!GridBaker)
	{
		UE_LOG(LogTemp, Warning, TEXT("BuildVoxelSpace fehlgeschlagen: GridBaker fehlt."));
		return;
	}

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

	const float VoxelSizeZCm = VoxelSizeZMeters * 100.0f;

	float MinRelevantZ = MinTerrainZ;
	float MaxRelevantZ = MaxTerrainZ;

	if (StartActor)
	{
		MinRelevantZ = FMath::Min(MinRelevantZ, StartActor->GetActorLocation().Z);
		MaxRelevantZ = FMath::Max(MaxRelevantZ, StartActor->GetActorLocation().Z);
	}

	if (GoalActor)
	{
		MinRelevantZ = FMath::Min(MinRelevantZ, GoalActor->GetActorLocation().Z);
		MaxRelevantZ = FMath::Max(MaxRelevantZ, GoalActor->GetActorLocation().Z);
	}

	VoxelMinWorld.X = HeightCache->GridMinWorld.X;
	VoxelMinWorld.Y = HeightCache->GridMinWorld.Y;
	VoxelMinWorld.Z = MinRelevantZ - (ExtraBottomLayers * VoxelSizeZCm);

	VoxelMaxWorld.X = HeightCache->GridMinWorld.X + (HeightCache->GridSize.X * HeightCache->CellSizeCm);
	VoxelMaxWorld.Y = HeightCache->GridMinWorld.Y + (HeightCache->GridSize.Y * HeightCache->CellSizeCm);
	VoxelMaxWorld.Z = MaxRelevantZ + (ExtraTopLayers * VoxelSizeZCm);

	const float HeightRangeZ = VoxelMaxWorld.Z - VoxelMinWorld.Z;
	ZLayerCount = FMath::Max(1, FMath::CeilToInt(HeightRangeZ / VoxelSizeZCm));

	UE_LOG(LogTemp, Display, TEXT("VoxelSpace gebaut: XY=%d x %d, ZLayers=%d, VoxelSizeZ=%.2fm"),
	       HeightCache->GridSize.X,
	       HeightCache->GridSize.Y,
	       ZLayerCount,
	       VoxelSizeZMeters);
}

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

	const float LocalX = WorldPos.X - HeightCache->GridMinWorld.X;
	const float LocalY = WorldPos.Y - HeightCache->GridMinWorld.Y;

	const int32 GridX = FMath::FloorToInt(LocalX / HeightCache->CellSizeCm);
	const int32 GridY = FMath::FloorToInt(LocalY / HeightCache->CellSizeCm);

	if (GridX < 0 || GridY < 0 || GridX >= HeightCache->GridSize.X || GridY >= HeightCache->GridSize.Y)
	{
		return false;
	}

	const float VoxelSizeZCm = VoxelSizeZMeters * 100.0f;
	const float LocalZ = WorldPos.Z - VoxelMinWorld.Z;
	const int32 GridZ = FMath::FloorToInt(LocalZ / VoxelSizeZCm);

	if (GridZ < 0 || GridZ >= ZLayerCount)
	{
		return false;
	}

	OutVoxel = FGridVoxelCoord(GridX, GridY, GridZ);
	return true;
}

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
	int32 StartZ = FMath::FloorToInt((WorldPos.Z - VoxelMinWorld.Z) / VoxelSizeZCm);
	StartZ = FMath::Clamp(StartZ, 0, ZLayerCount - 1);

	for (int32 Z = StartZ; Z < ZLayerCount; ++Z)
	{
		const FGridVoxelCoord Candidate(GridX, GridY, Z);
		if (!IsVoxelBlocked(Candidate))
		{
			OutVoxel = Candidate;
			return true;
		}
	}

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

FVector ASimple3DPathfinderActor::VoxelToWorldCenter(const FGridVoxelCoord& Voxel) const
{
	const float WorldX = HeightCache->GridMinWorld.X + ((float)Voxel.X + 0.5f) * HeightCache->CellSizeCm;
	const float WorldY = HeightCache->GridMinWorld.Y + ((float)Voxel.Y + 0.5f) * HeightCache->CellSizeCm;

	const float VoxelSizeZCm = VoxelSizeZMeters * 100.0f;
	const float WorldZ = VoxelMinWorld.Z + ((float)Voxel.Z + 0.5f) * VoxelSizeZCm;

	return FVector(WorldX, WorldY, WorldZ);
}

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

bool ASimple3DPathfinderActor::GetTerrainHeightCmAtWorldXY(float WorldX, float WorldY, float& OutTerrainHeightCm) const
{
	if (!HeightCache || !HeightCache->IsValid())
	{
		return false;
	}

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

	// Basisregel: nicht im Terrain
	if (VoxelCenter.Z <= TerrainHeightCm)
	{
		return true;
	}

	// Flugzeugprofil-Regeln
	if (!IsVoxelAllowedByFlightProfile(Voxel))
	{
		return true;
	}

	return false;
}

bool ASimple3DPathfinderActor::IsMoveAllowed(const FGridVoxelCoord& From, const FGridVoxelCoord& To) const
{
	if (!IsVoxelInsideBounds(To) || IsVoxelBlocked(To))
	{
		return false;
	}

	// Flugzeug-Leistungsgrenzen
	if (!IsMoveAllowedByFlightProfile(From, To))
	{
		return false;
	}

	if (!bPreventCornerCutting)
	{
		return true;
	}

	const int32 DX = To.X - From.X;
	const int32 DY = To.Y - From.Y;
	const int32 DZ = To.Z - From.Z;

	const int32 ChangedAxes =
		(DX != 0 ? 1 : 0) +
		(DY != 0 ? 1 : 0) +
		(DZ != 0 ? 1 : 0);

	if (ChangedAxes <= 1)
	{
		return true;
	}

	if (DX != 0)
	{
		const FGridVoxelCoord StepX(From.X + DX, From.Y, From.Z);
		if (IsVoxelBlocked(StepX))
		{
			return false;
		}
	}

	if (DY != 0)
	{
		const FGridVoxelCoord StepY(From.X, From.Y + DY, From.Z);
		if (IsVoxelBlocked(StepY))
		{
			return false;
		}
	}

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

float ASimple3DPathfinderActor::HeuristicCost(const FGridVoxelCoord& A, const FGridVoxelCoord& B) const
{
	return FVector::Distance(VoxelToWorldCenter(A), VoxelToWorldCenter(B));
}

float ASimple3DPathfinderActor::MovementCost(const FGridVoxelCoord& A, const FGridVoxelCoord& B) const
{
	return FVector::Distance(VoxelToWorldCenter(A), VoxelToWorldCenter(B));
}

void ASimple3DPathfinderActor::GetNeighbors(const FGridVoxelCoord& Center, TArray<FGridVoxelCoord>& OutNeighbors) const
{
	OutNeighbors.Reset();

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

bool ASimple3DPathfinderActor::FindBestOpenNode(
	const TSet<FGridVoxelCoord>& OpenSet,
	const TMap<FGridVoxelCoord, FGridVoxelNodeRecord>& Records,
	FGridVoxelCoord& OutBestNode
) const
{
	float BestF = TNumericLimits<float>::Max();
	bool bFound = false;

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

bool ASimple3DPathfinderActor::CanTravelDirect(const FVector& From, const FVector& To) const
{
	const float Distance = FVector::Distance(From, To);
	if (Distance <= KINDA_SMALL_NUMBER)
	{
		return true;
	}

	const float StepSize = HeightCache->CellSizeCm * 0.5f;
	const int32 NumSteps = FMath::Max(1, FMath::CeilToInt(Distance / StepSize));

	for (int32 Step = 0; Step <= NumSteps; ++Step)
	{
		const float Alpha = (float)Step / (float)NumSteps;
		const FVector Sample = FMath::Lerp(From, To, Alpha);

		float TerrainHeightCm = 0.0f;
		if (!GetTerrainHeightCmAtWorldXY(Sample.X, Sample.Y, TerrainHeightCm))
		{
			return false;
		}

		if (Sample.Z <= TerrainHeightCm)
		{
			return false;
		}
	}

	return true;
}

void ASimple3DPathfinderActor::SmoothPath()
{
	if (CurrentPathWorldPoints.Num() < 3)
	{
		return;
	}

	TArray<FVector> Smoothed;
	int32 CurrentIndex = 0;
	Smoothed.Add(CurrentPathWorldPoints[0]);

	while (CurrentIndex < CurrentPathWorldPoints.Num() - 1)
	{
		int32 FurthestReachable = CurrentIndex + 1;

		const int32 MaxTestIndex = FMath::Min(
			CurrentPathWorldPoints.Num() - 1,
			CurrentIndex + MaxSmoothSkipPoints
		);

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

void ASimple3DPathfinderActor::ReconstructPath(
	const TMap<FGridVoxelCoord, FGridVoxelNodeRecord>& Records,
	const FGridVoxelCoord& GoalVoxel
)
{
	CurrentPathWorldPoints.Reset();

	FGridVoxelCoord Current = GoalVoxel;

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

void ASimple3DPathfinderActor::FindPath()
{
	CurrentPathWorldPoints.Reset();

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

	TSet<FGridVoxelCoord> OpenSet;
	TMap<FGridVoxelCoord, FGridVoxelNodeRecord> Records;

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

		CurrentRecord->bClosed = true;

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

			const float TentativeG = CurrentRecord->G + MovementCost(Current, Neighbor);

			bool bShouldUpdate = false;

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

float ASimple3DPathfinderActor::GetAltitudeMetersASLFromWorldZ(float WorldZCm) const
{
	if (!HeightCache)
	{
		return 0.0f;
	}

	return (WorldZCm - HeightCache->SeaLevelWorldZCm) / 100.0f;
}

float ASimple3DPathfinderActor::GetTerrainHeightMetersASLAtCell(int32 X, int32 Y) const
{
	const float TerrainHeightCm = GetTerrainHeightCmAtCell(X, Y);
	if (TerrainHeightCm <= -1e20f || !HeightCache)
	{
		return -FLT_MAX;
	}

	return (TerrainHeightCm - HeightCache->SeaLevelWorldZCm) / 100.0f;
}

bool ASimple3DPathfinderActor::IsVoxelAllowedByFlightProfile(const FGridVoxelCoord& Voxel) const
{
	if (!FlightProfile)
	{
		return false;
	}

	if (!IsVoxelInsideBounds(Voxel))
	{
		return false;
	}

	const FVector VoxelCenter = VoxelToWorldCenter(Voxel);

	const float VoxelAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(VoxelCenter.Z);
	const float TerrainAltitudeMetersASL = GetTerrainHeightMetersASLAtCell(Voxel.X, Voxel.Y);

	if (TerrainAltitudeMetersASL <= -FLT_MAX / 2.0f)
	{
		return false;
	}

	// Maximale Flughöhe
	if (VoxelAltitudeMetersASL > FlightProfile->MaxAltitudeMetersASL)
	{
		return false;
	}

	// Mindestabstand zum Terrain
	const float ClearanceMeters = VoxelAltitudeMetersASL - TerrainAltitudeMetersASL;
	if (ClearanceMeters < FlightProfile->MinimumTerrainClearanceMeters)
	{
		return false;
	}

	return true;
}

bool ASimple3DPathfinderActor::IsMoveAllowedByFlightProfile(const FGridVoxelCoord& From, const FGridVoxelCoord& To) const
{
	if (!FlightProfile)
	{
		return false;
	}

	const FVector FromWorld = VoxelToWorldCenter(From);
	const FVector ToWorld = VoxelToWorldCenter(To);

	const float HorizontalDistanceCm = FVector2D::Distance(
		FVector2D(FromWorld.X, FromWorld.Y),
		FVector2D(ToWorld.X, ToWorld.Y)
	);

	const float HorizontalDistanceMeters = HorizontalDistanceCm / 100.0f;

	const float TravelTimeSeconds = FMath::Max(
		0.001f,
		HorizontalDistanceMeters / FlightProfile->CruiseSpeedMetersPerSecond
	);

	const float DeltaZMeters = (ToWorld.Z - FromWorld.Z) / 100.0f;

	if (DeltaZMeters > 0.0f)
	{
		const float MaxAllowedClimbMeters = FlightProfile->MaxClimbRateMetersPerSecond * TravelTimeSeconds;
		if (DeltaZMeters > MaxAllowedClimbMeters)
		{
			return false;
		}
	}

	if (DeltaZMeters < 0.0f)
	{
		const float MaxAllowedDescentMeters = FlightProfile->MaxDescentRateMetersPerSecond * TravelTimeSeconds;
		if (FMath::Abs(DeltaZMeters) > MaxAllowedDescentMeters)
		{
			return false;
		}
	}

	return true;
}