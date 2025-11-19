/**
 * PLY Point Cloud Collision Metadata Analyzer
 * 
 * INSTALLATION:
 * npm install ml-kmeans commander
 * 
 * USAGE:
 * node analyze-collision.js --input "/Users/me/export_last.ply" --voxel-size 0.25 --min-points 200 --k 8 --output bounds.json
 * 
 * ARGUMENTS:
 * --input        Absolute path to .ply file (required)
 * --voxel-size   Voxel size in meters (default: 0.25)
 * --min-points   Minimum points per voxel to keep (default: 200)
 * --k            Number of K-means clusters (default: 8)
 * --output       Output JSON path (default: ./bounds.json)
 * 
 * OUTPUT:
 * JSON file with global bounds, voxel AABBs, clusters, floor planes, and stair regions
 */

import fs from 'fs';
import path from 'path';
import { kmeans } from 'ml-kmeans';
import { Command } from 'commander';
import { fileURLToPath } from 'url';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

// ============================================
// CLI ARGUMENT PARSING
// ============================================

const program = new Command();
program
  .requiredOption('--input <path>', 'Absolute path to .ply file')
  .option('--voxel-size <size>', 'Voxel size in meters', '0.25')
  .option('--min-points <count>', 'Minimum points per voxel', '200')
  .option('--k <clusters>', 'Number of K-means clusters', '8')
  .option('--output <path>', 'Output JSON path', './bounds.json')
  .parse();

const opts = program.opts();
opts.voxelSize = parseFloat(opts.voxelSize);
opts.minPoints = parseInt(opts.minPoints);
opts.k = parseInt(opts.k);

// ============================================
// MAIN EXECUTION
// ============================================

async function main() {
  console.log('🔍 PLY Collision Metadata Analyzer');
  console.log('====================================\n');

  if (!fs.existsSync(opts.input)) {
    console.error(`❌ Error: File not found at ${opts.input}`);
    process.exit(1);
  }

  console.log(`📂 Input: ${opts.input}`);
  console.log(`📊 Voxel size: ${opts.voxelSize}m`);
  console.log(`📈 Min points: ${opts.minPoints}`);
  console.log(`🎯 Clusters: ${opts.k}\n`);

  const fileSize = (fs.statSync(opts.input).size / (1024 * 1024)).toFixed(2);
  console.log(`💾 File size: ${fileSize} MB\n`);

  console.log('⚙️  Parsing PLY file...');
  const points = parsePLYFile(opts.input);
  console.log(`✅ Loaded ${points.length.toLocaleString()} vertices\n`);

  if (points.length === 0) {
    console.error('❌ Error: No valid points found');
    process.exit(1);
  }

  console.log('📐 Computing global bounds...');
  const globalBounds = computeGlobalBounds(points);
  console.log(`   X: [${globalBounds.x[0]}, ${globalBounds.x[1]}] (${globalBounds.dimensions.width}m)`);
  console.log(`   Y: [${globalBounds.y[0]}, ${globalBounds.y[1]}] (${globalBounds.dimensions.height}m)`);
  console.log(`   Z: [${globalBounds.z[0]}, ${globalBounds.z[1]}] (${globalBounds.dimensions.depth}m)\n`);

  console.log('🧊 Building voxel grid...');
  const voxels = buildVoxelGrid(points, opts.voxelSize, opts.minPoints, globalBounds);
  console.log(`✅ Generated ${voxels.length} dense voxels\n`);

  console.log('🔗 Merging adjacent voxels...');
  const mergedVoxels = mergeAdjacentVoxels(voxels, opts.voxelSize);
  console.log(`✅ Merged into ${mergedVoxels.length} bounding boxes\n`);

  console.log('🏷️  Classifying voxels...');
  const classifiedVoxels = classifyVoxels(mergedVoxels);
  const wallCount = classifiedVoxels.filter(v => v.type === 'wall').length;
  const floorCount = classifiedVoxels.filter(v => v.type === 'floor').length;
  const objectCount = classifiedVoxels.filter(v => v.type === 'object').length;
  console.log(`   Walls: ${wallCount}, Floors: ${floorCount}, Objects: ${objectCount}\n`);

  console.log(`🎯 Computing ${opts.k} K-means clusters...`);
  const clusters = computeClusters(points, opts.k, opts.minPoints);
  console.log(`✅ Generated ${clusters.length} clusters\n`);

  console.log('🏢 Detecting floor planes...');
  const floors = detectFloorPlanes(points, globalBounds);
  console.log(`✅ Found ${floors.length} floor planes\n`);

  console.log('🪜 Detecting stairs...');
  const stairs = detectStairs(classifiedVoxels, globalBounds);
  console.log(`✅ Found ${stairs.length} stair regions\n`);

  const output = {
    metadata: {
      fileName: path.basename(opts.input),
      pointCount: points.length,
      voxelSize: opts.voxelSize,
      minPoints: opts.minPoints,
      clusterCount: opts.k,
      generatedAt: new Date().toISOString()
    },
    bounds: {
      x: globalBounds.x,
      y: globalBounds.y,
      z: globalBounds.z
    },
    center: globalBounds.center,
    dimensions: globalBounds.dimensions,
    voxels: classifiedVoxels,
    clusters: clusters,
    floors: floors,
    stairs: stairs
  };

  fs.writeFileSync(opts.output, JSON.stringify(output, null, 2));
  console.log(`💾 Saved to: ${opts.output}`);
  console.log('\n✨ Analysis complete!');
}

// ============================================
// PLY PARSER
// ============================================

function parsePLYFile(filePath) {
  const buffer = fs.readFileSync(filePath);
  const text = buffer.toString('utf-8', 0, Math.min(buffer.length, 100000));
  const lines = text.split('\n');

  let vertexCount = 0;
  let headerEndLine = 0;
  let isBinary = false;
  let format = 'ascii';

  for (let i = 0; i < lines.length; i++) {
    const line = lines[i].trim();

    if (line.startsWith('format binary_little_endian')) {
      isBinary = true;
      format = 'binary_little_endian';
    } else if (line.startsWith('format binary_big_endian')) {
      isBinary = true;
      format = 'binary_big_endian';
    } else if (line.startsWith('element vertex')) {
      vertexCount = parseInt(line.split(/\s+/)[2]);
    } else if (line === 'end_header') {
      headerEndLine = i;
      break;
    }
  }

  console.log(`   Format: ${format}`);
  console.log(`   Vertices: ${vertexCount.toLocaleString()}`);

  const points = [];

  if (isBinary) {
    const headerText = lines.slice(0, headerEndLine + 1).join('\n') + '\n';
    const headerBytes = Buffer.byteLength(headerText, 'utf-8');
    const dataBuffer = buffer.slice(headerBytes);

    const bytesPerVertex = 12;

    for (let i = 0; i < vertexCount; i++) {
      const offset = i * bytesPerVertex;

      if (offset + 12 <= dataBuffer.length) {
        const x = dataBuffer.readFloatLE(offset);
        const y = dataBuffer.readFloatLE(offset + 4);
        const z = dataBuffer.readFloatLE(offset + 8);

        if (isFinite(x) && isFinite(y) && isFinite(z)) {
          points.push({ x, y, z });
        }
      }
    }
  } else {
    const dataLines = text.split('\n').slice(headerEndLine + 1);

    for (let i = 0; i < Math.min(vertexCount, dataLines.length); i++) {
      const parts = dataLines[i].trim().split(/\s+/);

      if (parts.length >= 3) {
        const x = parseFloat(parts[0]);
        const y = parseFloat(parts[1]);
        const z = parseFloat(parts[2]);

        if (isFinite(x) && isFinite(y) && isFinite(z)) {
          points.push({ x, y, z });
        }
      }
    }
  }

  return points;
}

// ============================================
// GLOBAL BOUNDS
// ============================================

function computeGlobalBounds(points) {
  let minX = Infinity, maxX = -Infinity;
  let minY = Infinity, maxY = -Infinity;
  let minZ = Infinity, maxZ = -Infinity;

  for (const p of points) {
    if (p.x < minX) minX = p.x;
    if (p.x > maxX) maxX = p.x;
    if (p.y < minY) minY = p.y;
    if (p.y > maxY) maxY = p.y;
    if (p.z < minZ) minZ = p.z;
    if (p.z > maxZ) maxZ = p.z;
  }

  return {
    x: [round2(minX), round2(maxX)],
    y: [round2(minY), round2(maxY)],
    z: [round2(minZ), round2(maxZ)],
    center: [
      round2((minX + maxX) / 2),
      round2((minY + maxY) / 2),
      round2((minZ + maxZ) / 2)
    ],
    dimensions: {
      width: round2(maxX - minX),
      height: round2(maxY - minY),
      depth: round2(maxZ - minZ)
    }
  };
}

// ============================================
// VOXEL GRID
// ============================================

function buildVoxelGrid(points, voxelSize, minPoints, globalBounds) {
  const voxelMap = new Map();

  for (const p of points) {
    const vx = Math.floor(p.x / voxelSize);
    const vy = Math.floor(p.y / voxelSize);
    const vz = Math.floor(p.z / voxelSize);
    const key = `${vx},${vy},${vz}`;

    if (!voxelMap.has(key)) {
      voxelMap.set(key, {
        vx, vy, vz,
        points: [],
        bounds: {
          x: [vx * voxelSize, (vx + 1) * voxelSize],
          y: [vy * voxelSize, (vy + 1) * voxelSize],
          z: [vz * voxelSize, (vz + 1) * voxelSize]
        }
      });
    }

    voxelMap.get(key).points.push(p);
  }

  const voxels = [];
  let id = 0;

  for (const [key, voxel] of voxelMap) {
    if (voxel.points.length >= minPoints) {
      voxels.push({
        id: id++,
        voxelKey: key,
        vx: voxel.vx,
        vy: voxel.vy,
        vz: voxel.vz,
        bounds: voxel.bounds,
        pointCount: voxel.points.length
      });
    }
  }

  return voxels;
}

// ============================================
// MERGE ADJACENT VOXELS
// ============================================

function mergeAdjacentVoxels(voxels, voxelSize) {
  const visited = new Set();
  const merged = [];
  let mergedId = 0;

  for (const voxel of voxels) {
    if (visited.has(voxel.id)) continue;

    const cluster = [voxel];
    visited.add(voxel.id);
    const queue = [voxel];

    while (queue.length > 0) {
      const current = queue.shift();

      for (const other of voxels) {
        if (visited.has(other.id)) continue;

        const dx = Math.abs(current.vx - other.vx);
        const dy = Math.abs(current.vy - other.vy);
        const dz = Math.abs(current.vz - other.vz);

        if (dx <= 1 && dy <= 1 && dz <= 1 && (dx + dy + dz) <= 1) {
          visited.add(other.id);
          cluster.push(other);
          queue.push(other);
        }
      }
    }

    let minX = Infinity, maxX = -Infinity;
    let minY = Infinity, maxY = -Infinity;
    let minZ = Infinity, maxZ = -Infinity;
    let totalPoints = 0;

    for (const v of cluster) {
      if (v.bounds.x[0] < minX) minX = v.bounds.x[0];
      if (v.bounds.x[1] > maxX) maxX = v.bounds.x[1];
      if (v.bounds.y[0] < minY) minY = v.bounds.y[0];
      if (v.bounds.y[1] > maxY) maxY = v.bounds.y[1];
      if (v.bounds.z[0] < minZ) minZ = v.bounds.z[0];
      if (v.bounds.z[1] > maxZ) maxZ = v.bounds.z[1];
      totalPoints += v.pointCount;
    }

    merged.push({
      id: mergedId++,
      bounds: {
        x: [round2(minX), round2(maxX)],
        y: [round2(minY), round2(maxY)],
        z: [round2(minZ), round2(maxZ)]
      },
      pointCount: totalPoints,
      voxelCount: cluster.length
    });
  }

  return merged;
}

// ============================================
// VOXEL CLASSIFICATION
// ============================================

function classifyVoxels(voxels) {
  return voxels.map(v => {
    const width = v.bounds.x[1] - v.bounds.x[0];
    const height = v.bounds.y[1] - v.bounds.y[0];
    const depth = v.bounds.z[1] - v.bounds.z[0];

    const aspectRatio = Math.max(width, depth) / Math.min(width, depth);
    const verticalRatio = height / Math.max(width, depth);

    let type = 'object';

    if (height < 0.3 && Math.max(width, depth) > 1.0) {
      type = 'floor';
    } else if (verticalRatio > 2.0 && aspectRatio > 3.0) {
      type = 'wall';
    } else if (height > 1.5 && Math.max(width, depth) < 0.8) {
      type = 'wall';
    }

    return { ...v, type };
  });
}

// ============================================
// K-MEANS CLUSTERING
// ============================================

function computeClusters(points, k, minPoints) {
  const maxSampleSize = 10000;
  let sampledPoints = points;

  if (points.length > maxSampleSize) {
    const step = Math.floor(points.length / maxSampleSize);
    sampledPoints = points.filter((_, i) => i % step === 0);
    console.log(`   Using ${sampledPoints.length.toLocaleString()} sampled points`);
  }

  const data = sampledPoints.map(p => [p.x, p.y, p.z]);

  const result = kmeans(data, k, {
    initialization: 'kmeans++',
    maxIterations: 100
  });

  const clusterGroups = Array(k).fill(null).map(() => []);
  result.clusters.forEach((clusterId, idx) => {
    clusterGroups[clusterId].push(sampledPoints[idx]);
  });

  const clusters = [];

  for (let i = 0; i < k; i++) {
    const clusterPoints = clusterGroups[i];

    if (clusterPoints.length < minPoints) continue;

    let minX = Infinity, maxX = -Infinity;
    let minY = Infinity, maxY = -Infinity;
    let minZ = Infinity, maxZ = -Infinity;

    for (const p of clusterPoints) {
      if (p.x < minX) minX = p.x;
      if (p.x > maxX) maxX = p.x;
      if (p.y < minY) minY = p.y;
      if (p.y > maxY) maxY = p.y;
      if (p.z < minZ) minZ = p.z;
      if (p.z > maxZ) maxZ = p.z;
    }

    clusters.push({
      id: i,
      bounds: {
        x: [round2(minX), round2(maxX)],
        y: [round2(minY), round2(maxY)],
        z: [round2(minZ), round2(maxZ)]
      },
      center: [
        round2((minX + maxX) / 2),
        round2((minY + maxY) / 2),
        round2((minZ + maxZ) / 2)
      ],
      pointCount: clusterPoints.length
    });
  }

  return clusters;
}

// ============================================
// FLOOR PLANE DETECTION (RANSAC)
// ============================================

function detectFloorPlanes(points, globalBounds) {
  const floors = [];
  const yRange = globalBounds.y[1] - globalBounds.y[0];
  const sliceHeight = Math.max(0.1, yRange * 0.05);

  const yLevels = [];
  for (let y = globalBounds.y[0]; y <= globalBounds.y[1]; y += sliceHeight) {
    yLevels.push(y);
  }

  for (let i = 0; i < yLevels.length - 1; i++) {
    const yMin = yLevels[i];
    const yMax = yLevels[i + 1];

    const slicePoints = points.filter(p => p.y >= yMin && p.y < yMax);

    if (slicePoints.length < 100) continue;

    const avgY = slicePoints.reduce((sum, p) => sum + p.y, 0) / slicePoints.length;
    const variance = slicePoints.reduce((sum, p) => sum + Math.pow(p.y - avgY, 2), 0) / slicePoints.length;

    if (variance < 0.01) {
      let minX = Infinity, maxX = -Infinity;
      let minZ = Infinity, maxZ = -Infinity;

      for (const p of slicePoints) {
        if (p.x < minX) minX = p.x;
        if (p.x > maxX) maxX = p.x;
        if (p.z < minZ) minZ = p.z;
        if (p.z > maxZ) maxZ = p.z;
      }

      const area = (maxX - minX) * (maxZ - minZ);

      if (area > 1.0) {
        floors.push({
          normal: [0, 1, 0],
          bounds: {
            x: [round2(minX), round2(maxX)],
            y: [round2(avgY - 0.05), round2(avgY + 0.05)],
            z: [round2(minZ), round2(maxZ)]
          },
          confidence: round2(Math.min(1.0, slicePoints.length / 1000))
        });
      }
    }
  }

  return floors;
}

// ============================================
// STAIR DETECTION
// ============================================

function detectStairs(voxels, globalBounds) {
  const stairs = [];
  const floorVoxels = voxels.filter(v => v.type === 'floor');

  floorVoxels.sort((a, b) => {
    const aY = (a.bounds.y[0] + a.bounds.y[1]) / 2;
    const bY = (b.bounds.y[0] + b.bounds.y[1]) / 2;
    return aY - bY;
  });

  for (let i = 0; i < floorVoxels.length - 1; i++) {
    const current = floorVoxels[i];
    const next = floorVoxels[i + 1];

    const currentY = (current.bounds.y[0] + current.bounds.y[1]) / 2;
    const nextY = (next.bounds.y[0] + next.bounds.y[1]) / 2;
    const yDiff = nextY - currentY;

    if (yDiff > 0.1 && yDiff < 0.3) {
      const currentCenterX = (current.bounds.x[0] + current.bounds.x[1]) / 2;
      const currentCenterZ = (current.bounds.z[0] + current.bounds.z[1]) / 2;
      const nextCenterX = (next.bounds.x[0] + next.bounds.x[1]) / 2;
      const nextCenterZ = (next.bounds.z[0] + next.bounds.z[1]) / 2;

      const horizontalDist = Math.sqrt(
        Math.pow(nextCenterX - currentCenterX, 2) +
        Math.pow(nextCenterZ - currentCenterZ, 2)
      );

      if (horizontalDist > 0.2 && horizontalDist < 1.5) {
        const slope = yDiff / horizontalDist;

        if (slope > 0.1 && slope < 1.0) {
          const minX = Math.min(current.bounds.x[0], next.bounds.x[0]);
          const maxX = Math.max(current.bounds.x[1], next.bounds.x[1]);
          const minY = Math.min(current.bounds.y[0], next.bounds.y[0]);
          const maxY = Math.max(current.bounds.y[1], next.bounds.y[1]);
          const minZ = Math.min(current.bounds.z[0], next.bounds.z[0]);
          const maxZ = Math.max(current.bounds.z[1], next.bounds.z[1]);

          stairs.push({
            bounds: {
              x: [round2(minX), round2(maxX)],
              y: [round2(minY), round2(maxY)],
              z: [round2(minZ), round2(maxZ)]
            },
            slope: round2(slope),
            riseRun: round2(yDiff / horizontalDist)
          });
        }
      }
    }
  }

  return stairs;
}

// ============================================
// UTILITIES
// ============================================

function round2(value) {
  return Math.round(value * 100) / 100;
}

// ============================================
// RUN
// ============================================

main().catch(err => {
  console.error('\n❌ Error:', err.message);
  console.error(err.stack);
  process.exit(1);
});