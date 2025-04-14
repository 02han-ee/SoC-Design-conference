/*************************************************************
 * expanded_LiDAR_SLAM_realtimeLineChart_CSV_AllData_WITH_REAL_ICP.js
 *
 * (기존 코드에서 업그레이드/추가된 사항)
 * 1) ICP/GraphSLAM 모드에 "간단한 실제 ICP" 로직을 추가:
 *    - 직전 프레임의 LiDAR 스캔 vs 현재 프레임 스캔
 *    - 최근접 점 매칭 + SVD를 통해 2D rigid transform 도출
 *    - 로봇의 estimatedPos(x,y, heading)를 실제 회전·이동으로 보정
 * 2) 다른 기능(드로잉, DR/KF/PF, CSV, 차트 등)은 전부 유지
 *************************************************************/

let walls = [];
let robot;
let persistentPoints = [];
let showWalls = false;
let showPersistent = false;
let resolutionSlider;
let lidarModeButton;

let algoSelect; // <select> DOM
let currentAlgo = "DR"; // 기본값 DR

// FPS 측정
let lastTime = 0;
let avgFPS = 0;
let frameCount = 0;

// 오차(RMSE) 측정
let sumSquaredError = 0;
let numErrorSamples = 0;

// 전력 추정
let basePower = 5;
let rayCost = 0.05;
let pfCost = 0.3;
let icpCost = 0.6; // [ADDED for ICP] ICP 모드에서 추가 전력 가정
let estimatedPower = 0;

// 메모리 제한
let maxPersistentPoints = 3000;

// 데이터 로그(무제한)
let dataLog = []; // { fps, rmse, power, resolution, algo }

let exportButton;

function setup() {
  createCanvas(1200, 600);

  // (1) 랜덤 벽
  for (let i = 0; i < 5; i++) {
    let x1 = random(width / 2);
    let y1 = random(height);
    let x2 = random(width / 2);
    let y2 = random(height);
    walls.push(new Boundary(x1, y1, x2, y2));
  }
  // 왼쪽 영역 벽
  walls.push(new Boundary(0, 0, width / 2, 0));
  walls.push(new Boundary(width / 2, 0, width / 2, height));
  walls.push(new Boundary(width / 2, height, 0, height));
  walls.push(new Boundary(0, height, 0, 0));

  robot = new Robot(width / 4, height / 2);

  let toggleButton = createButton('Toggle Walls');
  toggleButton.position(10, 10);
  toggleButton.mousePressed(() => (showWalls = !showWalls));

  lidarModeButton = createButton('Toggle Persistent LiDAR (SLAM)');
  lidarModeButton.position(120, 10);
  lidarModeButton.mousePressed(() => (showPersistent = !showPersistent));

  resolutionSlider = createSlider(1, 15, 1, 1);
  resolutionSlider.position(360, 10);

  algoSelect = createSelect();
  algoSelect.position(520, 10);
  algoSelect.option('DR');
  algoSelect.option('KF');
  algoSelect.option('PF');
  // [ADDED] ICP 모드
  algoSelect.option('ICP/GraphSLAM');
  algoSelect.changed(() => {
    currentAlgo = algoSelect.value();
    sumSquaredError = 0;
    numErrorSamples = 0;
  });

  exportButton = createButton('Export CSV');
  exportButton.position(700, 10);
  exportButton.mousePressed(exportCSV);
}

function draw() {
  background(30);

  // FPS
  let now = performance.now();
  let delta = now - lastTime;
  lastTime = now;
  frameCount++;
  let instFPS = 1000 / (delta || 1);
  if (frameCount === 1) {
    avgFPS = instFPS;
  } else {
    avgFPS = 0.9 * avgFPS + 0.1 * instFPS;
  }

  // 벽 표시
  if (showWalls) {
    push();
    for (let wall of walls) {
      wall.show();
    }
    pop();
  }

  // 로봇 갱신
  robot.update(delta);
  let currentPoints = robot.look(walls);
  robot.show();

  // LiDAR 점 그리기
  fill(255, 0, 0);
  noStroke();
  for (let p of currentPoints) {
    ellipse(p.x, p.y, 5);
    if (showPersistent) persistentPoints.push(p.copy());
  }

  if (persistentPoints.length > maxPersistentPoints) {
    persistentPoints.splice(0, persistentPoints.length - maxPersistentPoints);
  }
  if (showPersistent) {
    fill(255, 100, 100, 80);
    noStroke();
    for (let p of persistentPoints) {
      ellipse(p.x, p.y, 3);
    }
  }

  // 알고리즘 추정 업데이트
  robot.updateEstimation(currentAlgo, delta, currentPoints); // [MODIFIED] currentPoints도 넘김

  // RMSE
  let dx = robot.pos.x - robot.estimatedPos.x;
  let dy = robot.pos.y - robot.estimatedPos.y;
  let distSq = dx * dx + dy * dy;
  sumSquaredError += distSq;
  numErrorSamples++;
  let rmse = sqrt(sumSquaredError / (numErrorSamples || 1));

  // 전력 추정
  let numRays = robot.rays.length;
  estimatedPower = basePower + numRays * rayCost;
  if (currentAlgo === 'PF') {
    estimatedPower += pfCost * numRays;
  } else if (currentAlgo === 'ICP/GraphSLAM') {
    estimatedPower += icpCost * numRays;
  }

  // 오른쪽 절반에 라인 차트
  drawTripleLineCharts(600, 0, 600, 600, dataLog);

  // 데이터 저장
  dataLog.push({
    fps: avgFPS,
    rmse: rmse,
    power: estimatedPower,
    resolution: resolutionSlider.value(),
    algo: currentAlgo,
  });

  // 좌측 상단
  fill(255);
  textSize(14);
  text(`FPS (avg): ${avgFPS.toFixed(2)}`, 10, 50);
  text(`Algo Mode: ${currentAlgo}`, 10, 70);
  text(`Resolution: ${resolutionSlider.value()}`, 10, 90);
  text(`RMSE: ${rmse.toFixed(2)}`, 10, 110);
  text(`Power: ${estimatedPower.toFixed(2)}`, 10, 130);
}

// ------------------------------------------------------------
// Line charts
// ------------------------------------------------------------
function drawTripleLineCharts(x, y, w, h, dataArray) {
  push();
  let areaH = h / 3;

  // FPS (top)
  push();
  translate(x, y);
  drawLineChartForProperty(0, 0, w, areaH, dataArray, "fps", 0, 60, color(0,255,0), "FPS");
  pop();

  // RMSE (middle)
  push();
  translate(x, y + areaH);
  drawLineChartForProperty(0, 0, w, areaH, dataArray, "rmse", 0, 100, color(0,100,255), "RMSE");
  pop();

  // Power (bottom)
  push();
  translate(x, y + 2*areaH);
  drawLineChartForProperty(0, 0, w, areaH, dataArray, "power", 0, 50, color(255,80,80), "Power");
  pop();

  pop();
}

function drawLineChartForProperty(x, y, w, h, dataArray, property, minVal, maxVal, col, label) {
  push();
  fill(60);
  noStroke();
  rect(x, y, w, h);

  stroke(col);
  strokeWeight(2);
  noFill();
  beginShape();
  for (let i=0; i<dataArray.length; i++){
    let px = map(i, 0, dataArray.length-1, x, x+w);
    let py = map(dataArray[i][property], minVal, maxVal, y+h, y);
    vertex(px, py);
  }
  endShape();

  noStroke();
  fill(255);
  textSize(12);
  text(label, x+5, y+15);
  text(minVal, x+5, y+h-5);
  text(maxVal, x+5, y+15);
  pop();
}

// ------------------------------------------------------------
// CSV Export
// ------------------------------------------------------------
function exportCSV() {
  let table = new p5.Table();
  table.addColumn("frame");
  table.addColumn("fps");
  table.addColumn("rmse");
  table.addColumn("power");
  table.addColumn("resolution");
  table.addColumn("algo");

  for (let i = 0; i < dataLog.length; i++){
    let row = table.addRow();
    row.setString("frame", i);
    row.setString("fps", nf(dataLog[i].fps, 2, 2));
    row.setString("rmse", nf(dataLog[i].rmse, 2, 2));
    row.setString("power", nf(dataLog[i].power, 2, 2));
    row.setString("resolution", dataLog[i].resolution);
    row.setString("algo", dataLog[i].algo);
  }
  saveTable(table, "dataLog.csv", "csv");
}

// ------------------------------------------------------------
// Boundary, Ray: 기존 그대로
// ------------------------------------------------------------
class Boundary {
  constructor(x1, y1, x2, y2) {
    this.a = createVector(x1, y1);
    this.b = createVector(x2, y2);
  }
  show() {
    stroke(255);
    line(this.a.x, this.a.y, this.b.x, this.b.y);
  }
}

class Ray {
  constructor(pos, angle) {
    this.pos = pos;
    this.dir = p5.Vector.fromAngle(angle);
  }

  cast(wall, maxDist) {
    const x1 = wall.a.x, y1 = wall.a.y;
    const x2 = wall.b.x, y2 = wall.b.y;
    const x3 = this.pos.x, y3 = this.pos.y;
    const x4 = this.pos.x + this.dir.x*maxDist, y4 = this.pos.y + this.dir.y*maxDist;

    const den = (x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4);
    if (den === 0) return null;
    const t = ((x1 - x3)*(y3 - y4) - (y1 - y3)*(x3 - x4)) / den;
    const u = -((x1 - x2)*(y1 - y3) - (y1 - y2)*(x1 - x3)) / den;
    if (t>0 && t<1 && u>0 && u<1) {
      const pt = createVector();
      pt.x = x1 + t*(x2 - x1);
      pt.y = y1 + t*(y2 - y1);
      return pt;
    }
    return null;
  }
}

// ------------------------------------------------------------
// Robot
//   - DR, KF, PF: 기존과 동일
//   - ICP/GraphSLAM: 실제 ICP 1회차용 로직 추가
// ------------------------------------------------------------
class Robot {
  constructor(x, y) {
    this.pos = createVector(x, y);
    this.rays = [];
    this.heading = 0;
    this.vel = createVector();
    this.maxDist = 150;
    this.colliding = false;

    // 추정 위치
    this.estimatedPos = createVector(x, y);

    // Dead Reckoning
    this.estimatedVel_DR = createVector(0, 0);

    // PF
    this.particleCount = 1000;

    // [ADDED for ICP] 이전 프레임의 LiDAR 포인트 저장
    // (월드 좌표계 상)
    this.lastScanPoints = [];
  }

  update(deltaTimeMS) {
    let newPos = this.pos.copy();
    if (keyIsDown(LEFT_ARROW)) {
      this.heading -= 0.03;
    } else if (keyIsDown(RIGHT_ARROW)) {
      this.heading += 0.03;
    }
    if (keyIsDown(UP_ARROW)) {
      this.vel = p5.Vector.fromAngle(this.heading);
      this.vel.setMag(2);
      newPos.add(this.vel);
    } else if (keyIsDown(DOWN_ARROW)) {
      this.vel = p5.Vector.fromAngle(this.heading);
      this.vel.setMag(-2);
      newPos.add(this.vel);
    }
    this.colliding = this.collide(newPos);
    if (!this.colliding) {
      this.pos = newPos;
    }

    // 레이 업데이트
    this.rays = [];
    const resolution = resolutionSlider.value();
    for (let a = -PI/4; a <= PI/4; a += radians(resolution)) {
      this.rays.push(new Ray(this.pos, a + this.heading));
    }
  }

  // [MODIFIED]: currentScan(= 이번 프레임 LiDAR 점)도 받음
  updateEstimation(algoMode, deltaTimeMS, currentScan) {
    let dt = deltaTimeMS / 1000;
    if (!isFinite(dt) || dt <= 0) dt = 0.016;

    switch (algoMode) {
      case 'DR': {
        // Dead Reckoning
        this.estimatedVel_DR = p5.Vector.fromAngle(this.heading);
        this.estimatedVel_DR.setMag(2);
        this.estimatedPos.add(this.estimatedVel_DR.copy().mult(dt*60));
        break;
      }

      case 'KF': {
        for (let i=0; i<100000; i++) { /* dummy */ }
        let kfGain = 0.01;
        let errorVecKF = p5.Vector.sub(this.pos, this.estimatedPos);
        errorVecKF.mult(kfGain);
        this.estimatedPos.add(errorVecKF);
        break;
      }

      case 'PF': {
        let pfParticles = this.particleCount;
        let sumX=0, sumY=0;
        for (let i=0; i<pfParticles; i++){
          let angleRand = random(TWO_PI);
          let distRand = random(0,50);
          sumX += this.pos.x + cos(angleRand)*distRand;
          sumY += this.pos.y + sin(angleRand)*distRand;
        }
        sumX /= pfParticles;
        sumY /= pfParticles;
        this.estimatedPos.x = lerp(this.estimatedPos.x, sumX, 0.05);
        this.estimatedPos.y = lerp(this.estimatedPos.y, sumY, 0.05);
        break;
      }

      case 'ICP/GraphSLAM': {
        // 충분한 유효점이 없으면 ICP 업데이트를 건너뜁니다.
        if (this.lastScanPoints.length < 5 || !currentScan || currentScan.length < 5) {
          // ICP 업데이트 없음
        } else {
          let maxIterations = 3;         // 반복 횟수 감소
          let inlierThreshold = 30;        // 30px 임계값: outlier 제거
          for (let iter = 0; iter < maxIterations; iter++) {
            // 계산 부하를 줄이기 위해 현재 스캔에서 최대 20개 점만 사용
            let sampleSize = Math.min(20, currentScan.length);
            let sampledCurrent = currentScan.slice(0, sampleSize);
            let pairs = [];
            // 각 sampledCurrent 점에 대해, lastScanPoints에서 가장 가까운 점을 찾고, 거리가 inlierThreshold 이내이면 대응 쌍으로 채택
            for (let pNow of sampledCurrent) {
              let bestDist = Infinity;
              let bestPt = null;
              for (let pOld of this.lastScanPoints) {
                let dd = p5.Vector.dist(pNow, pOld);
                if (dd < bestDist) {
                  bestDist = dd;
                  bestPt = pOld;
                }
              }
              if (bestDist < inlierThreshold) {
                pairs.push({ now: pNow, old: bestPt });
              }
            }
            
            // 충분한 대응 쌍이 없으면 반복 종료
            if (pairs.length < 5) {
              break;
            }
            
            // 각 대응 쌍의 centroid 계산
            let sumNow = createVector(0, 0);
            let sumOld = createVector(0, 0);
            for (let pr of pairs) {
              sumNow.add(pr.now);
              sumOld.add(pr.old);
            }
            sumNow.div(pairs.length);
            sumOld.div(pairs.length);
            
            // 분산행렬 H (2x2) 계산
            let M11 = 0, M12 = 0, M21 = 0, M22 = 0;
            for (let pr of pairs) {
              let nowC = p5.Vector.sub(pr.now, sumNow);
              let oldC = p5.Vector.sub(pr.old, sumOld);
              M11 += nowC.x * oldC.x;
              M12 += nowC.x * oldC.y;
              M21 += nowC.y * oldC.x;
              M22 += nowC.y * oldC.y;
            }
            
            // 간단한 SVD 대신 ICP 전용 공식: θ = atan2(M21 - M12, M11 + M22)
            let theta = Math.atan2(M21 - M12, M11 + M22);
            let cosT = Math.cos(theta);
            let sinT = Math.sin(theta);
            // 회전 후 lastScanPoints의 centroid 계산
            let rotatedSumOld = createVector(cosT * sumOld.x - sinT * sumOld.y,
                                             sinT * sumOld.x + cosT * sumOld.y);
            // translation 계산: translation = sumNow - rotatedSumOld
            let translation = createVector(sumNow.x - rotatedSumOld.x,
                                           sumNow.y - rotatedSumOld.y);
            // 로봇의 추정 위치 및 heading 보정
            this.estimatedPos.add(translation);
            this.heading += theta;
          }
        }
        break;
      }

    }

    // [마지막 단계] 이번 프레임 스캔을 다음 프레임에 이용하기 위해 저장
    // (ICP 모드 아니어도 계속 저장 -> PF->ICP 전환시 안정 작동)
    if (currentScan && currentScan.length>0) {
      this.lastScanPoints = currentScan.slice(); // shallow copy
    }
  }

  collide(newPos) {
    for (let wall of walls) {
      let d = distToSegment(newPos, wall.a, wall.b);
      if (d < 10) return true;
    }
    return false;
  }

  look(walls) {
    stroke(100,255,100,80);
    let points = [];
    for (let ray of this.rays) {
      let closest = null;
      let record = this.maxDist;
      for (let wall of walls) {
        const pt = ray.cast(wall, this.maxDist);
        if (pt) {
          let dd = p5.Vector.dist(this.pos, pt);
          if (dd < record){
            record = dd;
            closest = pt;
          }
        }
      }
      line(
        this.pos.x, this.pos.y,
        closest ? closest.x : ray.pos.x+ray.dir.x*this.maxDist,
        closest ? closest.y : ray.pos.y+ray.dir.y*this.maxDist
      );
      if (closest) points.push(closest);
    }
    return points;
  }

  show() {
    fill(this.colliding?'red':'white');
    ellipse(this.pos.x, this.pos.y, 12);
    stroke(255);
    strokeWeight(2);
    line(
      this.pos.x, this.pos.y,
      this.pos.x + Math.cos(this.heading)*20,
      this.pos.y + Math.sin(this.heading)*20
    );

    // 추정 위치 (파랑)
    push();
    fill(0,100,255,180);
    noStroke();
    ellipse(this.estimatedPos.x, this.estimatedPos.y, 8);
    pop();
  }
}

// ------------------------------------------------------------
function distToSegment(p, v, w){
  const l2 = p5.Vector.dist(v, w)**2;
  if (l2===0) return p5.Vector.dist(p, v);
  let t = max(0, min(1, ((p.x - v.x)*(w.x - v.x) + (p.y - v.y)*(w.y - v.y)) / l2));
  let proj = createVector(v.x + t*(w.x - v.x), v.y + t*(w.y - v.y));
  return p5.Vector.dist(p, proj);
}
