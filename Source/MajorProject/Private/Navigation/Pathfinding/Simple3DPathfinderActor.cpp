// Fill out your copyright notice in the Description page of Project Settings.


#include "Navigation/Pathfinding/Simple3DPathfinderActor.h"
#include "Navigation/Grid/VoxelGridBaker.h"
#include "Navigation/Grid/VoxelHeightCache.h"
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

	// Phase 1 Logik:
	// Alles im oder unter dem Terrain ist blockiert
	return VoxelCenter.Z <= TerrainHeightCm;
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

				if (IsVoxelInsideBounds(Neighbor))
				{
					OutNeighbors.Add(Neighbor);
				}
			}
		}
	}
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

	TArray<FGridVoxelCoord> OpenSet;
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

		int32 BestIndex = INDEX_NONE;
		float BestF = TNumericLimits<float>::Max();

		for (int32 i = 0; i < OpenSet.Num(); ++i)
		{
			const FGridVoxelCoord& Candidate = OpenSet[i];
			const FGridVoxelNodeRecord* Record = Records.Find(Candidate);

			if (!Record || Record->bClosed)
			{
				continue;
			}

			if (Record->F < BestF)
			{
				BestF = Record->F;
				BestIndex = i;
			}
		}

		if (BestIndex == INDEX_NONE)
		{
			break;
		}

		const FGridVoxelCoord Current = OpenSet[BestIndex];
		OpenSet.RemoveAtSwap(BestIndex);

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
			if (IsVoxelBlocked(Neighbor))
			{
				continue;
			}

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
			false,
			DebugDrawLifetime,
			0,
			DebugLineThickness
		);

		DrawDebugSphere(
			GetWorld(),
			CurrentPathWorldPoints[i],
			40.0f,
			8,
			FColor::Yellow,
			false,
			DebugDrawLifetime,
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
		false,
		DebugDrawLifetime,
		0,
		2.0f
	);
}

