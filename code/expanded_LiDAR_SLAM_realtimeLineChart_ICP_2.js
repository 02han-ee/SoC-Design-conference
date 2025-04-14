/*************************************************************
 * expanded_LiDAR_SLAM_realtimeLineChart_CSV_AllData_WITH_REAL_ICP.js
 *
 * (기존 코드에서 업그레이드/추가된 사항)
 * 1) ICP/GraphSLAM 모드에 "간단한 실제 ICP" 로직을 추가:
 *    - 직전 프레임의 LiDAR 스캔 vs 현재 프레임 스캔
 *    - 최근접 점 매칭 + SVD를 통해 2D rigid transform 도출
 *    - 로봇의 estimatedPos(x,y, heading)를 실제 회전·이동으로 보정
 * 2) 다른 기능(드로잉, DR/KF/PF, CSV, 차트 등)은 전부 유지
 * 
 * KF 필터 수정(25.4.14.오후 6PM14M 기준)
 * PF 필터 수정(25.4.14.오후 6PM44M 기준)
 **** PF 필터 문제 있음, 잘 안 돌아감, 완벽한 코드 아님.
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
        // KF 변수가 아직 없다면 초기화 (상태: [x, y, theta], P: 3x3 공분산 행렬)
        if (!this.kf) {
          this.kf = {
            x: this.estimatedPos.x,
            y: this.estimatedPos.y,
            theta: this.heading,
            P: [
              [1, 0, 0],
              [0, 1, 0],
              [0, 0, 1]
            ]
          };
        }

        let dt = deltaTimeMS / 1000;
        if (!isFinite(dt) || dt <= 0) dt = 0.016;

        // 제어 입력: 키 입력(UP/DOWN, LEFT/RIGHT)을 이용해 선형 속도 및 회전 속도 결정
        let v = 0;
        if (keyIsDown(UP_ARROW)) {
          v = 2;
        } else if (keyIsDown(DOWN_ARROW)) {
          v = -2;
        }
        let omega = 0;
        if (keyIsDown(LEFT_ARROW)) {
          omega = -0.03;
        } else if (keyIsDown(RIGHT_ARROW)) {
          omega = 0.03;
        }

        // 예측 단계: 
        // 상태 업데이트: x_k = x_{k-1} + v*dt*cos(theta), y_k = y_{k-1} + v*dt*sin(theta), theta_k = theta_{k-1} + omega*dt
        let theta = this.kf.theta;
        let predictedState = {
          x: this.kf.x + v * dt * Math.cos(theta),
          y: this.kf.y + v * dt * Math.sin(theta),
          theta: theta + omega * dt
        };

        // 상태 전파의 선형화: Jacobian A = ∂f/∂x
        let A = [
          [1, 0, -v * dt * Math.sin(theta)],
          [0, 1,  v * dt * Math.cos(theta)],
          [0, 0, 1]
        ];

        // 프로세스 노이즈 Q (단순화를 위해 작게 설정)
        let Q = [
          [0.1, 0, 0],
          [0, 0.1, 0],
          [0, 0, 0.05]
        ];

        // 예측 공분산: P_pred = A * P * A^T + Q
        let P = this.kf.P;
        // 먼저 A * P (3x3)
        let AP = [
          [
            A[0][0]*P[0][0] + A[0][1]*P[1][0] + A[0][2]*P[2][0],
            A[0][0]*P[0][1] + A[0][1]*P[1][1] + A[0][2]*P[2][1],
            A[0][0]*P[0][2] + A[0][1]*P[1][2] + A[0][2]*P[2][2]
          ],
          [
            A[1][0]*P[0][0] + A[1][1]*P[1][0] + A[1][2]*P[2][0],
            A[1][0]*P[0][1] + A[1][1]*P[1][1] + A[1][2]*P[2][1],
            A[1][0]*P[0][2] + A[1][1]*P[1][2] + A[1][2]*P[2][2]
          ],
          [
            A[2][0]*P[0][0] + A[2][1]*P[1][0] + A[2][2]*P[2][0],
            A[2][0]*P[0][1] + A[2][1]*P[1][1] + A[2][2]*P[2][1],
            A[2][0]*P[0][2] + A[2][1]*P[1][2] + A[2][2]*P[2][2]
          ]
        ];
        // P_pred = (A*P)*A^T + Q
        let P_pred = [[0,0,0],[0,0,0],[0,0,0]];
        for (let i = 0; i < 3; i++) {
          for (let j = 0; j < 3; j++) {
            for (let k = 0; k < 3; k++) {
              // A[j][k] == (A^T)[k][j]
              P_pred[i][j] += AP[i][k] * A[j][k];
            }
            P_pred[i][j] += Q[i][j];
          }
        }

        // 관측 단계:
        // 이번 프레임 LiDAR 스캔(currentScan)의 centroid를 관측치 z (x, y)로 계산 (heading 측정은 없음)
        let z = null;
        if (currentScan && currentScan.length > 0) {
          let sumX = 0, sumY = 0;
          for (let p of currentScan) {
            sumX += p.x;
            sumY += p.y;
          }
          let n = currentScan.length;
          z = {
            x: sumX / n,
            y: sumY / n
          };
        }
        // 측정 행렬 H (상태에서 관측치로의 매핑: H*x = [x, y])
        let H = [
          [1, 0, 0],
          [0, 1, 0]
        ];

        // 측정 노이즈 R (단순화를 위해 설정)
        let R_cov = [
          [5, 0],
          [0, 5]
        ];

        if (z) {
          // 혁신(innovation): y = z - H * predictedState
          let hx = [predictedState.x, predictedState.y]; // H*x 예측은 단순히 위치 값
          let innovation = {
            x: z.x - hx[0],
            y: z.y - hx[1]
          };

          // 혁신 공분산: S = H * P_pred * H^T + R
          // H*P_pred (2x3) : 각 행은 P_pred의 첫번째, 두번째 행으로 구성
          let HP = [
            [ P_pred[0][0], P_pred[0][1], P_pred[0][2] ],
            [ P_pred[1][0], P_pred[1][1], P_pred[1][2] ]
          ];
          // S = HP * H^T + R (H^T의 형태를 고려하면 S는 2x2)
          let S = [
            [ HP[0][0], HP[0][1] ],
            [ HP[1][0], HP[1][1] ]
          ];
          S[0][0] += R_cov[0][0];
          S[0][1] += R_cov[0][1];
          S[1][0] += R_cov[1][0];
          S[1][1] += R_cov[1][1];

          // Kalman Gain: K = P_pred * H^T * inv(S)
          // P_pred * H^T는 3x2 행렬 (H^T는 [[1,0],[0,1],[0,0]])
          let PHt = [
            [ P_pred[0][0], P_pred[0][1] ],
            [ P_pred[1][0], P_pred[1][1] ],
            [ P_pred[2][0], P_pred[2][1] ]
          ];
          // 2x2 행렬 S의 역행렬 계산
          let detS = S[0][0]*S[1][1] - S[0][1]*S[1][0];
          let invS = [
            [ S[1][1] / detS, -S[0][1] / detS ],
            [ -S[1][0] / detS, S[0][0] / detS ]
          ];
          // K = PHt * invS (3x2 행렬 곱셈)
          let K = [[0,0],[0,0],[0,0]];
          for (let i = 0; i < 3; i++){
            for (let j = 0; j < 2; j++){
              for (let k = 0; k < 2; k++){
                K[i][j] += PHt[i][k] * invS[k][j];
              }
            }
          }

          // 상태 업데이트: x = predictedState + K * innovation
          let updatedState = {
            x: predictedState.x + K[0][0]*innovation.x + K[0][1]*innovation.y,
            y: predictedState.y + K[1][0]*innovation.x + K[1][1]*innovation.y,
            theta: predictedState.theta // 관측치가 없으므로 theta는 예측값 유지
          };

          // 공분산 업데이트: P = (I - K*H) * P_pred
          // K*H는 3x3 행렬 (H 확장은 [ [1,0,0], [0,1,0] ]에 맞춰)
          let KH = [
            [K[0][0], K[0][1], 0],
            [K[1][0], K[1][1], 0],
            [K[2][0], K[2][1], 0]
          ];
          let I = [[1,0,0],[0,1,0],[0,0,1]];
          let I_KH = [
            [ I[0][0]-KH[0][0], I[0][1]-KH[0][1], I[0][2]-KH[0][2] ],
            [ I[1][0]-KH[1][0], I[1][1]-KH[1][1], I[1][2]-KH[1][2] ],
            [ I[2][0]-KH[2][0], I[2][1]-KH[2][1], I[2][2]-KH[2][2] ]
          ];
          let newP = [[0,0,0],[0,0,0],[0,0,0]];
          for (let i = 0; i < 3; i++){
            for (let j = 0; j < 3; j++){
              for (let k = 0; k < 3; k++){
                newP[i][j] += I_KH[i][k] * P_pred[k][j];
              }
            }
          }
          // KF 상태 및 공분산 보정
          this.kf.x = updatedState.x;
          this.kf.y = updatedState.y;
          this.kf.theta = updatedState.theta;
          this.kf.P = newP;
        } else {
          // 관측치가 없는 경우, 예측 상태를 그대로 사용
          this.kf.x = predictedState.x;
          this.kf.y = predictedState.y;
          this.kf.theta = predictedState.theta;
          this.kf.P = P_pred;
        }

        // KF를 통해 갱신된 상태로 추정 위치를 보정
        this.estimatedPos.x = this.kf.x;
        this.estimatedPos.y = this.kf.y;
        // (옵션) 필요시 heading도 KF 상태로 업데이트: this.heading = this.kf.theta;
        break;
      }


      case 'PF': {
        // 파티클 배열 초기화 (최초 한 번)
        if (!this.particles) {
          this.particles = [];
          for (let i = 0; i < this.particleCount; i++) {
            this.particles.push({
              x: this.estimatedPos.x,
              y: this.estimatedPos.y,
              theta: this.heading,
              weight: 1.0
            });
          }
        }
        
        // 제어 입력 추출 (로봇과 동일하게)
        let v = 0;
        if (keyIsDown(UP_ARROW)) {
          v = 2;
        } else if (keyIsDown(DOWN_ARROW)) {
          v = -2;
        }
        let omega = 0;
        if (keyIsDown(LEFT_ARROW)) {
          omega = -0.03;
        } else if (keyIsDown(RIGHT_ARROW)) {
          omega = 0.03;
        }
        let dt = deltaTimeMS / 1000;
        if (!isFinite(dt) || dt <= 0) dt = 0.016;
        
        // 모션 업데이트: 각 파티클에 대해 제어 입력에 따른 이동 + 노이즈 추가
        const motionNoiseTrans = 0.5; // 위치 노이즈 표준편차
        const motionNoiseRot = 0.01;  // 회전 노이즈 표준편차
        for (let p of this.particles) {
          p.theta += omega * dt + randomGaussian(0, motionNoiseRot);
          p.x += v * dt * cos(p.theta) + randomGaussian(0, motionNoiseTrans);
          p.y += v * dt * sin(p.theta) + randomGaussian(0, motionNoiseTrans);
        }
        
        // PF 측정 업데이트: 계산량 줄이기 위해 각 파티클 당 5개 샘플만 사용
        function simulateScan(particle) {
          let scanPoints = [];
          const maxDist = 150;
          const numSamples = 5;  // 샘플 수 축소
          for (let i = 0; i < numSamples; i++) {
            // -PI/4부터 PI/4까지 균등하게 샘플링
            let a = -PI/4 + i * ((PI/2) / (numSamples - 1));
            let rayAngle = particle.theta + a;
            let rayOrigin = createVector(particle.x, particle.y);
            let rayDir = p5.Vector.fromAngle(rayAngle);
            let closest = null;
            let record = maxDist;
            for (let wall of walls) {
              const x1 = wall.a.x, y1 = wall.a.y;
              const x2 = wall.b.x, y2 = wall.b.y;
              const x3 = rayOrigin.x, y3 = rayOrigin.y;
              const x4 = rayOrigin.x + rayDir.x * maxDist, y4 = rayOrigin.y + rayDir.y * maxDist;
              const den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
              if (den === 0) continue;
              const t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den;
              const u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / den;
              if (t > 0 && t < 1 && u > 0 && u < 1) {
                const pt = createVector(x1 + t * (x2 - x1), y1 + t * (y2 - y1));
                let d = p5.Vector.dist(rayOrigin, pt);
                if (d < record) {
                  record = d;
                  closest = pt;
                }
              }
            }
            if (closest) {
              scanPoints.push(closest);
            } else {
              // 벽에 닿지 않으면 최대 거리에 해당하는 점 사용
              scanPoints.push(createVector(rayOrigin.x + rayDir.x * maxDist, rayOrigin.y + rayDir.y * maxDist));
            }
          }
          return scanPoints;
        }
        
        // 실제 LiDAR 스캔(currentScan)으로부터 centroid 계산
        let actualCentroid = null;
        if (currentScan && currentScan.length > 0) {
          let sumX = 0, sumY = 0;
          for (let pt of currentScan) {
            sumX += pt.x;
            sumY += pt.y;
          }
          actualCentroid = { x: sumX / currentScan.length, y: sumY / currentScan.length };
        }
        
        // 각 파티클마다 예상 스캔의 centroid와 실제 centroid 사이 오차를 이용해 가중치 계산
        const sensorSigma = 20; // 센서 노이즈 표준편차
        let weightSum = 0;
        for (let p of this.particles) {
          let predictedScan = simulateScan(p);
          let sumX = 0, sumY = 0;
          for (let pt of predictedScan) {
            sumX += pt.x;
            sumY += pt.y;
          }
          let predCentroid = { x: sumX / predictedScan.length, y: sumY / predictedScan.length };
          let error = dist(predCentroid.x, predCentroid.y, actualCentroid.x, actualCentroid.y);
          p.weight = Math.exp(- (error * error) / (2 * sensorSigma * sensorSigma));
          weightSum += p.weight;
        }
        
        // 가중치 정규화
        for (let p of this.particles) {
          p.weight /= weightSum;
        }
        
        // 순차적 리샘플링 (Systematic Resampling) 방식 사용
        let newParticles = [];
        let N = this.particles.length;
        // 누적 가중치 배열 계산
        let cumsum = [];
        let cum = 0;
        for (let p of this.particles) {
          cum += p.weight;
          cumsum.push(cum);
        }
        // 시작 위치: [0, 1/N)
        let start = random(0, 1/N);
        let positions = [];
        for (let i = 0; i < N; i++) {
          positions.push(start + i / N);
        }
        let index = 0;
        for (let pos of positions) {
          while (pos > cumsum[index]) {
            index++;
          }
          // 복제 후 가중치는 초기화
          newParticles.push({ x: this.particles[index].x, y: this.particles[index].y, theta: this.particles[index].theta, weight: 1.0 });
        }
        this.particles = newParticles;
        
        // 파티클 분포의 평균을 추정 위치로 업데이트 (조금씩 보정)
        let meanX = 0, meanY = 0;
        for (let p of this.particles) {
          meanX += p.x;
          meanY += p.y;
        }
        meanX /= this.particles.length;
        meanY /= this.particles.length;
        this.estimatedPos.x = lerp(this.estimatedPos.x, meanX, 0.1);
        this.estimatedPos.y = lerp(this.estimatedPos.y, meanY, 0.1);
        
        break;
      }



      case 'ICP/GraphSLAM': {
        // 충분한 스캔 데이터가 없으면 ICP 업데이트를 생략합니다.
        if (this.lastScanPoints.length < 5 || !currentScan || currentScan.length < 5) {
          break;
        } else {
          let maxIterations = 5;         // 최대 ICP 반복 횟수
          let inlierThreshold = 30;        // 30px 임계값: outlier 제거
          for (let iter = 0; iter < maxIterations; iter++) {
            // 1) ICP 매칭: 현재 스캔의 모든 점에 대해, lastScanPoints 중 가장 가까운 점 찾기
            let pairs = [];
            for (let pNow of currentScan) {
              let bestDist = Infinity;
              let bestPt = null;
              for (let pOld of this.lastScanPoints) {
                let dd = p5.Vector.dist(pNow, pOld);
                if (dd < bestDist) {
                  bestDist = dd;
                  bestPt = pOld;
                }
              }
              if (bestDist < inlierThreshold) {  // inlier로 간주할 임계값
                pairs.push({ now: pNow, old: bestPt });
              }
            }
            
            // 대응 쌍이 충분하지 않으면 반복 종료
            if (pairs.length < 5) {
              break;
            }
            
            // 2) 각 대응 쌍의 centroid 계산 (현재 스캔과 이전 스캔 각각)
            let sumNow = createVector(0, 0);
            let sumOld = createVector(0, 0);
            for (let pr of pairs) {
              sumNow.add(pr.now);
              sumOld.add(pr.old);
            }
            sumNow.div(pairs.length);
            sumOld.div(pairs.length);
            
            // 3) 분산행렬(2x2) H 계산  
            //    H = Σ [ (pNow - centroid_now) * (pOld - centroid_old)^T ]
            let M11 = 0, M12 = 0, M21 = 0, M22 = 0;
            for (let pr of pairs) {
              let nowC = p5.Vector.sub(pr.now, sumNow);
              let oldC = p5.Vector.sub(pr.old, sumOld);
              M11 += nowC.x * oldC.x;
              M12 += nowC.x * oldC.y;
              M21 += nowC.y * oldC.x;
              M22 += nowC.y * oldC.y;
            }
            
            // 4) 회전 각도 추정
            //    ICP 2D 공식: theta = atan2(M21 - M12, M11 + M22)
            let theta = Math.atan2(M21 - M12, M11 + M22);
            
            // 5) 회전 및 평행이동 보정 (pivot 방식을 사용)
            //    (xOld, yOld)를 R*(xOld - centroid_old) + centroid_now 로 매핑한다고 가정
            //    R = [cosθ, -sinθ; sinθ, cosθ]
            this.heading += theta;
            let cosT = Math.cos(theta);
            let sinT = Math.sin(theta);
            // pivot으로 사용하는 이전 대응의 centroid
            let oldPivot = sumOld;
            let newPivot = sumNow;
            // 회전된 pivot: R*(oldPivot)
            let Rx = cosT * oldPivot.x - sinT * oldPivot.y;
            let Ry = sinT * oldPivot.x + cosT * oldPivot.y;
            // translation: newPivot - R*(oldPivot)
            let shiftX = newPivot.x - Rx;
            let shiftY = newPivot.y - Ry;
            this.estimatedPos.x += shiftX;
            this.estimatedPos.y += shiftY;
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
