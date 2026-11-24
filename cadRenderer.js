// Helper to draw an arrow line on canvas
function drawCanvasArrow(ctx, x1, y1, x2, y2, color, lineWidth, headLen = 9) {
  const angle = Math.atan2(y2 - y1, x2 - x1);
  ctx.save();
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = lineWidth;
  ctx.lineCap = 'round';
  ctx.beginPath();
  ctx.moveTo(x1, y1);
  ctx.lineTo(x2, y2);
  ctx.stroke();

  ctx.beginPath();
  ctx.moveTo(x2, y2);
  ctx.lineTo(x2 - headLen * Math.cos(angle - Math.PI / 6), y2 - headLen * Math.sin(angle - Math.PI / 6));
  ctx.lineTo(x2 - headLen * Math.cos(angle + Math.PI / 6), y2 - headLen * Math.sin(angle + Math.PI / 6));
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}

// Draw curved blue sketch arrow (matching the user's hand-drawn blue ink arrows!)
function drawCurvedTrajectoryArrow(ctx, startX, startY, ctrlX, ctrlY, endX, endY, color, label) {
  ctx.save();
  ctx.strokeStyle = color;
  ctx.lineWidth = 3.2;
  ctx.shadowColor = color;
  ctx.shadowBlur = 8;
  ctx.setLineDash([6, 4]);
  ctx.beginPath();
  ctx.moveTo(startX, startY);
  ctx.quadraticCurveTo(ctrlX, ctrlY, endX, endY);
  ctx.stroke();
  ctx.setLineDash([]);

  // Arrowhead at end
  const tangentAngle = Math.atan2(endY - ctrlY, endX - ctrlX);
  const headLen = 11;
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.moveTo(endX, endY);
  ctx.lineTo(endX - headLen * Math.cos(tangentAngle - Math.PI / 6), endY - headLen * Math.sin(tangentAngle - Math.PI / 6));
  ctx.lineTo(endX - headLen * Math.cos(tangentAngle + Math.PI / 6), endY - headLen * Math.sin(tangentAngle + Math.PI / 6));
  ctx.closePath();
  ctx.fill();

  if (label) {
    ctx.shadowBlur = 0;
    ctx.font = 'bold 11px Prompt, sans-serif';
    ctx.fillStyle = (color === '#ef4444' || color === '#fca5a5') ? color : '#93c5fd';
    ctx.textAlign = 'center';
    ctx.fillText(label, ctrlX, ctrlY - 8);
  }
  ctx.restore();
}

// Compute V-Frame Geometry in Side-View Canvas Coordinates
// Base Pivot (px, py), theta1Deg (angle of 680mm arm from horizontal left/right), slideMm (0..370mm)
function computeVFramePoints(pivotX, pivotY, theta1Deg, slideMm, scaleMmToPx) {
  const L1_px = 680 * scaleMmToPx;
  const L2_px = 370 * scaleMmToPx;

  // In the CAD drawing, the 680mm arm rises up and to the right from the pivot (e.g. 55 deg from horizontal)
  // And when sweeping back in full-arm mode, theta1 increases toward 120 deg (sweeping left/up)
  const rad1 = (theta1Deg * Math.PI) / 180.0;
  const apexX = pivotX + Math.cos(rad1) * L1_px;
  const apexY = pivotY - Math.sin(rad1) * L1_px;

  // The 370mm slotted link hangs down-right from the apex at an included V-angle
  // In CAD side view, the 370mm rail points downward & slightly right (~ -75 deg from horizontal)
  const rad2 = rad1 - (128 * Math.PI) / 180.0;
  const railEndX = apexX + Math.cos(rad2) * L2_px;
  const railEndY = apexY - Math.sin(rad2) * L2_px;

  // Slider position S2 (measured from bottom of 370mm rail toward apex: 0mm = bottom, 370mm = apex)
  const clampedSlide = Math.max(30, Math.min(350, slideMm));
  const fracFromApex = 1.0 - (clampedSlide / 370.0);
  const sliderX = apexX + Math.cos(rad2) * (L2_px * fracFromApex);
  const sliderY = apexY - Math.sin(rad2) * (L2_px * fracFromApex);

  return {
    pivotX,
    pivotY,
    apexX,
    apexY,
    railEndX,
    railEndY,
    sliderX,
    sliderY,
    rad1,
    rad2,
    L1_px,
    L2_px
  };
}

// Exact 2-Bone Analytical Inverse Kinematics (Law of Cosines) — ZERO limb stretching!
function solveTwoBoneIK(rootX, rootY, targetX, targetY, L1, L2, bendDir) {
  let dx = targetX - rootX;
  let dy = targetY - rootY;
  let dist = Math.hypot(dx, dy);
  const minDist = Math.abs(L1 - L2) + 4;
  const maxDist = (L1 + L2) - 2;

  let effX = targetX;
  let effY = targetY;
  let clamped = false;

  if (dist > maxDist && dist > 0.001) {
    const r = maxDist / dist;
    effX = rootX + dx * r;
    effY = rootY + dy * r;
    dx = effX - rootX;
    dy = effY - rootY;
    dist = maxDist;
    clamped = true;
  } else if (dist < minDist) {
    const ang = dist > 0.001 ? Math.atan2(dy, dx) : -Math.PI * 0.25;
    effX = rootX + Math.cos(ang) * minDist;
    effY = rootY + Math.sin(ang) * minDist;
    dx = effX - rootX;
    dy = effY - rootY;
    dist = minDist;
    clamped = true;
  }

  const cosA = Math.max(-1, Math.min(1, (L1 * L1 + dist * dist - L2 * L2) / (2 * L1 * dist)));
  const baseAngle = Math.atan2(dy, dx);
  const offsetAngle = Math.acos(cosA);
  const jointAngle = baseAngle + bendDir * offsetAngle;

  return {
    jointX: rootX + Math.cos(jointAngle) * L1,
    jointY: rootY + Math.sin(jointAngle) * L1,
    effX,
    effY,
    clamped
  };
}

// Helper: Draw a realistic tapered anatomical limb segment with 3D cylindrical shading
function drawTaperedLimb(ctx, x1, y1, x2, y2, r1, r2, baseColor, highlightColor, strokeColor) {
  const dx = x2 - x1;
  const dy = y2 - y1;
  const len = Math.hypot(dx, dy);
  if (len < 1) return;
  const ang = Math.atan2(dy, dx);

  ctx.save();
  ctx.translate(x1, y1);
  ctx.rotate(ang);

  const grad = ctx.createLinearGradient(0, -r1, 0, r1);
  grad.addColorStop(0, highlightColor);
  grad.addColorStop(0.55, baseColor);
  grad.addColorStop(1, '#1e293b');

  ctx.fillStyle = grad;
  ctx.strokeStyle = strokeColor;
  ctx.lineWidth = 1.2;
  ctx.beginPath();
  ctx.arc(0, 0, r1, Math.PI * 0.5, Math.PI * 1.5);
  ctx.lineTo(len, -r2);
  ctx.arc(len, 0, r2, -Math.PI * 0.5, Math.PI * 0.5);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  ctx.restore();
}

// Helper: Draw realistic brushed-steel structural extrusion beam with bevels & bolt holes
function drawSteelBeam(ctx, x1, y1, x2, y2, thickness, highlighted, showLeadScrew = false) {
  const dx = x2 - x1;
  const dy = y2 - y1;
  const len = Math.hypot(dx, dy);
  if (len < 2) return;
  const ang = Math.atan2(dy, dx);
  const halfT = thickness * 0.5;

  ctx.save();
  ctx.translate(x1, y1);
  ctx.rotate(ang);

  // Drop shadow for 2.5D depth
  ctx.shadowColor = 'rgba(0, 0, 0, 0.55)';
  ctx.shadowBlur = 8;
  ctx.shadowOffsetY = 3;

  // Brushed steel linear gradient across beam width
  const steelGrad = ctx.createLinearGradient(0, -halfT, 0, halfT);
  if (highlighted) {
    steelGrad.addColorStop(0, '#f8fafc');
    steelGrad.addColorStop(0.25, '#cbd5e1');
    steelGrad.addColorStop(0.55, '#64748b');
    steelGrad.addColorStop(0.85, '#94a3b8');
    steelGrad.addColorStop(1, '#334155');
  } else {
    steelGrad.addColorStop(0, '#94a3b8');
    steelGrad.addColorStop(0.35, '#475569');
    steelGrad.addColorStop(0.75, '#334155');
    steelGrad.addColorStop(1, '#1e293b');
  }

  ctx.fillStyle = steelGrad;
  ctx.strokeStyle = highlighted ? '#e2e8f0' : '#475569';
  ctx.lineWidth = 1.1;
  ctx.beginPath();
  ctx.roundRect(-halfT * 0.5, -halfT, len + halfT, thickness, halfT * 0.6);
  ctx.fill();
  ctx.shadowBlur = 0;
  ctx.shadowOffsetY = 0;
  ctx.stroke();

  // Inner T-Slot / Machined Groove
  const slotW = showLeadScrew ? thickness * 0.44 : thickness * 0.28;
  ctx.fillStyle = '#090f1c';
  ctx.beginPath();
  ctx.roundRect(len * 0.08, -slotW * 0.5, len * 0.84, slotW, slotW * 0.5);
  ctx.fill();

  if (showLeadScrew) {
    // Metallic threaded lead-screw rod inside the 370mm linear actuator slot
    ctx.strokeStyle = '#94a3b8';
    ctx.lineWidth = 2.2;
    ctx.beginPath();
    ctx.moveTo(len * 0.1, 0);
    ctx.lineTo(len * 0.9, 0);
    ctx.stroke();

    // Thread ridges
    ctx.strokeStyle = '#334155';
    ctx.lineWidth = 1;
    for (let sx = len * 0.12; sx < len * 0.88; sx += 5) {
      ctx.beginPath();
      ctx.moveTo(sx, -slotW * 0.38);
      ctx.lineTo(sx + 2, slotW * 0.38);
      ctx.stroke();
    }
  } else if (highlighted) {
    // Subtle cyan anodized accent line along active 680mm steel beam
    ctx.strokeStyle = 'rgba(56, 189, 248, 0.75)';
    ctx.lineWidth = 1.2;
    ctx.beginPath();
    ctx.moveTo(len * 0.12, 0);
    ctx.lineTo(len * 0.88, 0);
    ctx.stroke();
  }

  ctx.restore();
}

// Helper: Draw realistic industrial rotary servo motor + rotating gear hub at pivot
function drawRotaryServoMotor(ctx, cx, cy, thetaDeg, isActive, isEmergency, blinkOn, label) {
  ctx.save();
  ctx.translate(cx, cy);

  // 1) Motor Casing Mounted Behind/Below Pivot (Industrial ribbed housing)
  const bodyGrad = ctx.createLinearGradient(-22, -8, 12, 22);
  bodyGrad.addColorStop(0, '#334155');
  bodyGrad.addColorStop(0.5, '#1e293b');
  bodyGrad.addColorStop(1, '#090f1c');
  ctx.fillStyle = bodyGrad;
  ctx.strokeStyle = isEmergency && blinkOn ? '#ef4444' : (isActive ? '#38bdf8' : '#64748b');
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.roundRect(-22, -4, 30, 22, 5);
  ctx.fill();
  ctx.stroke();

  // Cooling fins on motor housing
  ctx.strokeStyle = '#475569';
  ctx.lineWidth = 1.2;
  for (let fx = -18; fx <= -6; fx += 4) {
    ctx.beginPath();
    ctx.moveTo(fx, -1);
    ctx.lineTo(fx, 15);
    ctx.stroke();
  }

  // 2) Brushed Aluminum Gearbox Flange Ring
  const hubGrad = ctx.createRadialGradient(-3, -3, 2, 0, 0, 14);
  hubGrad.addColorStop(0, '#e2e8f0');
  hubGrad.addColorStop(0.55, '#64748b');
  hubGrad.addColorStop(1, '#1e293b');
  ctx.fillStyle = hubGrad;
  ctx.strokeStyle = isEmergency ? (blinkOn ? '#ef4444' : '#fca5a5') : (isActive ? '#22d3ee' : '#94a3b8');
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(0, 0, 13.5, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();

  // 3) Rotating Gear / Drive Spokes (rotates live with thetaDeg!)
  ctx.save();
  ctx.rotate((-thetaDeg * Math.PI) / 180.0);
  ctx.strokeStyle = '#0f172a';
  ctx.lineWidth = 2.2;
  for (let i = 0; i < 4; i++) {
    const a = (i * Math.PI) / 2;
    ctx.beginPath();
    ctx.moveTo(Math.cos(a) * 3.5, Math.sin(a) * 3.5);
    ctx.lineTo(Math.cos(a) * 10.5, Math.sin(a) * 10.5);
    ctx.stroke();
  }
  // Brass center drive shaft + hex nut
  ctx.fillStyle = '#f59e0b';
  ctx.beginPath();
  ctx.arc(0, 0, 4.2, 0, Math.PI * 2);
  ctx.fill();
  ctx.strokeStyle = '#fef3c7';
  ctx.lineWidth = 1;
  ctx.stroke();
  ctx.restore();

  // 4) Live Status LED on Motor Housing
  ctx.fillStyle = isEmergency
    ? (blinkOn ? '#ef4444' : '#7f1d1d')
    : (isActive ? '#10b981' : '#f59e0b');
  ctx.beginPath();
  ctx.arc(-16, 12, 2.6, 0, Math.PI * 2);
  ctx.fill();

  // 5) Motor Label Pill (M1 / M2)
  ctx.fillStyle = 'rgba(15, 23, 42, 0.9)';
  ctx.strokeStyle = isEmergency ? '#ef4444' : (isActive ? '#38bdf8' : '#475569');
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.roundRect(-24, 19, 48, 13, 4);
  ctx.fill();
  ctx.stroke();
  ctx.fillStyle = isEmergency ? '#fca5a5' : (isActive ? '#7dd3fc' : '#cbd5e1');
  ctx.font = 'bold 8.5px monospace';
  ctx.textAlign = 'center';
  ctx.fillText(label, 0, 28.5);

  ctx.restore();
}

// Helper: Draw a small clean pill label so CAD numbers never collide or become unreadable
function drawCadLabelPill(ctx, x, y, text, textColor, borderColor = 'rgba(71,85,105,0.7)') {
  ctx.save();
  ctx.font = 'bold 9.5px monospace';
  const tw = ctx.measureText(text).width;
  const padX = 5;
  const h = 15;
  ctx.fillStyle = 'rgba(9, 15, 28, 0.86)';
  ctx.strokeStyle = borderColor;
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.roundRect(x - tw * 0.5 - padX, y - h * 0.5, tw + padX * 2, h, 4);
  ctx.fill();
  ctx.stroke();
  ctx.fillStyle = textColor;
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText(text, x, y + 0.5);
  ctx.restore();
}

function drawSideCadBlueprint(k) {
  const canvas = document.getElementById('cadSideCanvas');
  if (!canvas) return;
  const dpr = window.devicePixelRatio || 1;
  if (canvas.width === 0 && canvas.parentElement.clientWidth > 0) {
    canvas.width = canvas.parentElement.clientWidth * dpr;
    canvas.height = canvas.parentElement.clientHeight * dpr;
  }
  const ctx = canvas.getContext('2d');
  const w = canvas.width;
  const h = canvas.height;
  if (w === 0 || h === 0) return;

  ctx.save();
  ctx.scale(dpr, dpr);
  const cw = w / dpr;
  const ch = h / dpr;

  const isHwEmergency =
    (typeof hwSensorState !== 'undefined' && hwSensorState.connected && hwSensorState.isEmergency) ||
    (typeof dashboardEmergency !== 'undefined' && dashboardEmergency.active);
  const emergencyBlinkOn = Math.floor(Date.now() / 220) % 2 === 0;
  const emergencyRed = emergencyBlinkOn ? '#ef4444' : '#fca5a5';

  // 1. Studio Engineering Backdrop + Blueprint Grid + Subtle Floor Horizon
  const bgGrad = ctx.createLinearGradient(0, 0, 0, ch);
  bgGrad.addColorStop(0, '#050a14');
  bgGrad.addColorStop(0.75, '#0b1426');
  bgGrad.addColorStop(1, '#070d19');
  ctx.fillStyle = bgGrad;
  ctx.fillRect(0, 0, cw, ch);

  ctx.strokeStyle = 'rgba(56, 189, 248, 0.08)';
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let x = 0; x < cw; x += 28) { ctx.moveTo(x, 0); ctx.lineTo(x, ch); }
  for (let y = 0; y < ch; y += 28) { ctx.moveTo(0, y); ctx.lineTo(cw, y); }
  ctx.stroke();

  // Auto-fitted coordinate mapping for the 1900mm bed so the 680mm V-Arm apex NEVER clips!
  const bedY = ch - 84;
  const maxScaleByW = (cw * 0.86) / 1900.0;
  const maxScaleByH = Math.max(0.14, (bedY - 36) / 705.0);
  const scaleMm = Math.min(maxScaleByW, maxScaleByH);
  const bedWidthPx = 1900.0 * scaleMm;
  const bedMarginX = (cw - bedWidthPx) * 0.5;
  const bedThickPx = Math.max(13, 46 * scaleMm);
  const mattressH = Math.max(15, 54 * scaleMm);

  // Floor reflection line under bed legs
  const floorY = bedY + bedThickPx + 22;
  ctx.strokeStyle = 'rgba(148, 163, 184, 0.22)';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(bedMarginX - 24, floorY);
  ctx.lineTo(bedMarginX + bedWidthPx + 24, floorY);
  ctx.stroke();

  // Compute the two V-Frame Mechanisms
  const leftPivotX = bedMarginX + 410 * scaleMm;
  const leftPivotY = bedY - 2;
  const upperVF = computeVFramePoints(leftPivotX, leftPivotY, k.upperTheta1, k.upperSlide2, scaleMm);

  const rightPivotX = bedMarginX + 1220 * scaleMm;
  const rightPivotY = bedY - 2;
  const lowerVF = computeVFramePoints(rightPivotX, rightPivotY, k.lowerTheta1, k.lowerSlide2, scaleMm);

  const upperActive = (kinSelectedPart === 'arm' || kinSelectedPart === 'elbow' || kinSelectedPart === 'all4');
  const lowerActive = (kinSelectedPart === 'leg' || kinSelectedPart === 'knee' || kinSelectedPart === 'all4');

  // 2. INDUSTRIAL STEEL BED FRAME (Heavy-Duty Brushed Steel I-Beam + Legs + Cross-Bracing)
  // Vertical Steel Support Legs + Rubber Leveling Pads
  const legPositionsMm = [65, 410, 820, 1220, 1835];
  // Horizontal lower steel cross-bar tying the frame together
  ctx.fillStyle = '#334155';
  ctx.strokeStyle = '#64748b';
  ctx.lineWidth = 1;
  ctx.fillRect(bedMarginX + 65 * scaleMm, bedY + bedThickPx + 10, (1835 - 65) * scaleMm, 4);
  ctx.strokeRect(bedMarginX + 65 * scaleMm, bedY + bedThickPx + 10, (1835 - 65) * scaleMm, 4);

  legPositionsMm.forEach(mm => {
    const lx = bedMarginX + mm * scaleMm;
    // Steel leg post
    const legGrad = ctx.createLinearGradient(lx - 4, 0, lx + 4, 0);
    legGrad.addColorStop(0, '#94a3b8');
    legGrad.addColorStop(0.5, '#475569');
    legGrad.addColorStop(1, '#1e293b');
    ctx.fillStyle = legGrad;
    ctx.fillRect(lx - 3.5, bedY + bedThickPx, 7, 19);
    ctx.strokeStyle = '#64748b';
    ctx.lineWidth = 0.8;
    ctx.strokeRect(lx - 3.5, bedY + bedThickPx, 7, 19);
    // Rubber foot pad
    ctx.fillStyle = '#0f172a';
    ctx.strokeStyle = '#94a3b8';
    ctx.beginPath();
    ctx.roundRect(lx - 7, floorY - 3, 14, 3.5, 1.5);
    ctx.fill();
    ctx.stroke();
  });

  // Main 1900mm Horizontal Steel Frame Rail (Beveled Metallic Beam)
  const railGrad = ctx.createLinearGradient(0, bedY, 0, bedY + bedThickPx);
  railGrad.addColorStop(0, '#cbd5e1');
  railGrad.addColorStop(0.22, '#64748b');
  railGrad.addColorStop(0.7, '#334155');
  railGrad.addColorStop(1, '#0f172a');
  ctx.fillStyle = railGrad;
  ctx.strokeStyle = '#94a3b8';
  ctx.lineWidth = 1.4;
  ctx.beginPath();
  ctx.roundRect(bedMarginX, bedY, bedWidthPx, bedThickPx, 3);
  ctx.fill();
  ctx.stroke();

  // Inner structural groove & hex rivets along the steel bed beam
  ctx.strokeStyle = 'rgba(15, 23, 42, 0.7)';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(bedMarginX + 8, bedY + bedThickPx * 0.5);
  ctx.lineTo(bedMarginX + bedWidthPx - 8, bedY + bedThickPx * 0.5);
  ctx.stroke();

  // Steel Bed Hinge Mount Plates (770 | 1030 | 1420)
  [770, 1030, 1420].forEach(mm => {
    const jx = bedMarginX + mm * scaleMm;
    ctx.fillStyle = '#1e293b';
    ctx.strokeStyle = '#cbd5e1';
    ctx.lineWidth = 1.6;
    ctx.beginPath();
    ctx.arc(jx, bedY + bedThickPx * 0.5, 5.5, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = '#94a3b8';
    ctx.beginPath();
    ctx.arc(jx, bedY + bedThickPx * 0.5, 2, 0, Math.PI * 2);
    ctx.fill();
  });

  // Medical Mattress (4-section padded dark slate with subtle seams)
  const matGrad = ctx.createLinearGradient(0, bedY - mattressH, 0, bedY);
  matGrad.addColorStop(0, '#334155');
  matGrad.addColorStop(0.35, '#1e293b');
  matGrad.addColorStop(1, '#0f172a');
  ctx.fillStyle = matGrad;
  ctx.strokeStyle = '#64748b';
  ctx.lineWidth = 1.3;
  ctx.beginPath();
  ctx.roundRect(bedMarginX + 2, bedY - mattressH, bedWidthPx - 4, mattressH, [7, 7, 1, 1]);
  ctx.fill();
  ctx.stroke();

  // Mattress 4-section articulation seams (770 | 1030 | 1400)
  ctx.strokeStyle = 'rgba(15, 23, 42, 0.85)';
  ctx.lineWidth = 1.5;
  [770, 1030, 1400].forEach(mm => {
    const sx = bedMarginX + mm * scaleMm;
    ctx.beginPath();
    ctx.moveTo(sx, bedY - mattressH + 2);
    ctx.lineTo(sx, bedY - 1);
    ctx.stroke();
  });

  // Soft Ergonomic Pillow
  const pillowX = bedMarginX + 55 * scaleMm;
  const pillowW = 215 * scaleMm;
  const pillowH = Math.max(11, 38 * scaleMm);
  const pilGrad = ctx.createLinearGradient(0, bedY - mattressH - pillowH, 0, bedY - mattressH);
  pilGrad.addColorStop(0, '#f1f5f9');
  pilGrad.addColorStop(1, '#94a3b8');
  ctx.fillStyle = pilGrad;
  ctx.beginPath();
  ctx.roundRect(pillowX, bedY - mattressH - pillowH + 2, pillowW, pillowH, 7);
  ctx.fill();

  // 3. BACKGROUND STEEL V-FRAME LINKAGES (drawn behind patient limbs for realistic depth)
  // 4. REALISTIC ANATOMICAL HUMAN PATIENT (Rigid 2-Bone IK — ZERO Limb Stretching!)
  const matTopY = bedY - mattressH;
  const headR = Math.max(11, 68 * scaleMm);
  const headCenterX = bedMarginX + 175 * scaleMm;
  const headCenterY = matTopY - pillowH - headR * 0.45 + 3;
  const shoulderX = bedMarginX + 390 * scaleMm;
  const shoulderY = matTopY - Math.max(14, 52 * scaleMm);
  const hipX = bedMarginX + 910 * scaleMm;
  const hipY = matTopY - Math.max(13, 46 * scaleMm);

  // Neck
  drawTaperedLimb(ctx, headCenterX + headR * 0.65, headCenterY + headR * 0.35, shoulderX - 18 * scaleMm, shoulderY + 2, headR * 0.42, headR * 0.48, '#cbd5e1', '#f1f5f9', '#64748b');

  // Contoured Human Torso (Chest -> Waist -> Pelvis in clinical slate-teal shirt)
  const torsoGrad = ctx.createLinearGradient(0, shoulderY - 16, 0, matTopY);
  torsoGrad.addColorStop(0, '#475569');
  torsoGrad.addColorStop(0.45, '#334155');
  torsoGrad.addColorStop(1, '#1e293b');
  ctx.fillStyle = torsoGrad;
  ctx.strokeStyle = '#94a3b8';
  ctx.lineWidth = 1.4;
  ctx.beginPath();
  ctx.moveTo(shoulderX - 28 * scaleMm, matTopY);
  ctx.quadraticCurveTo(shoulderX - 24 * scaleMm, shoulderY - 16, shoulderX + 90 * scaleMm, shoulderY - 15);
  ctx.quadraticCurveTo((shoulderX + hipX) * 0.5, shoulderY - 8, hipX + 30 * scaleMm, hipY - 12);
  ctx.quadraticCurveTo(hipX + 55 * scaleMm, matTopY - 6, hipX + 42 * scaleMm, matTopY);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();

  // Head Silhouette with subtle 3D shading
  const headGrad = ctx.createRadialGradient(headCenterX - 3, headCenterY - 4, 2, headCenterX, headCenterY, headR);
  headGrad.addColorStop(0, '#f8fafc');
  headGrad.addColorStop(0.65, '#cbd5e1');
  headGrad.addColorStop(1, '#64748b');
  ctx.fillStyle = headGrad;
  ctx.strokeStyle = '#94a3b8';
  ctx.lineWidth = 1.2;
  ctx.beginPath();
  ctx.arc(headCenterX, headCenterY, headR, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();

  // --- RIGID 2-BONE IK FOR LOWER LIMB (Thigh = 450mm constant, Shin = 430mm constant — NEVER STRETCHES!) ---
  const L_THIGH = 450 * scaleMm;
  const L_SHIN = 430 * scaleMm;
  const ankleGoalX = lowerVF.sliderX;
  const ankleGoalY = lowerVF.sliderY + 12 * scaleMm;
  // Knee bends UPWARD (-1 in canvas Y-down coordinates)
  const legIK = solveTwoBoneIK(hipX, hipY, ankleGoalX, ankleGoalY, L_THIGH, L_SHIN, -1);
  const kneeX = legIK.jointX;
  const kneeY = legIK.jointY;
  const ankleX = legIK.effX;
  const ankleY = legIK.effY;

  // If slider slightly exceeds rigid leg reach, draw flexible nylon support strap between carriage and ankle cuff
  if (legIK.clamped) {
    ctx.strokeStyle = '#94a3b8';
    ctx.lineWidth = 2;
    ctx.setLineDash([3, 2]);
    ctx.beginPath();
    ctx.moveTo(lowerVF.sliderX, lowerVF.sliderY);
    ctx.lineTo(ankleX, ankleY);
    ctx.stroke();
    ctx.setLineDash([]);
  }

  const legBase = lowerActive ? '#10b981' : '#64748b';
  const legHi = lowerActive ? '#6ee7b7' : '#cbd5e1';
  const legStroke = lowerActive ? '#a7f3d0' : '#94a3b8';
  const thighR = Math.max(6.5, 29 * scaleMm);
  const kneeR = Math.max(5.2, 22 * scaleMm);
  const ankleR = Math.max(4.0, 15 * scaleMm);

  // Draw Rigid Thigh & Rigid Shin
  drawTaperedLimb(ctx, hipX, hipY, kneeX, kneeY, thighR, kneeR, legBase, legHi, legStroke);
  drawTaperedLimb(ctx, kneeX, kneeY, ankleX, ankleY, kneeR * 0.92, ankleR, legBase, legHi, legStroke);

  // Rigid Anatomical Foot (145mm constant, perpendicular to shin angle)
  const shinAng = Math.atan2(ankleY - kneeY, ankleX - kneeX);
  const footAng = shinAng - Math.PI * 0.48;
  const L_FOOT = 145 * scaleMm;
  const toeX = ankleX + Math.cos(footAng) * L_FOOT;
  const toeY = ankleY + Math.sin(footAng) * L_FOOT;
  drawTaperedLimb(ctx, ankleX, ankleY, toeX, toeY, ankleR * 1.05, ankleR * 0.72, '#334155', '#94a3b8', '#cbd5e1');

  // Hip & Knee Joint Caps
  [ [hipX, hipY], [kneeX, kneeY] ].forEach(([jx, jy]) => {
    ctx.fillStyle = lowerActive ? '#059669' : '#475569';
    ctx.strokeStyle = lowerActive ? '#d1fae5' : '#cbd5e1';
    ctx.lineWidth = 1.4;
    ctx.beginPath();
    ctx.arc(jx, jy, 4.5, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
  });

  // --- RIGID 2-BONE IK FOR UPPER LIMB (Upper Arm = 335mm constant, Forearm = 305mm constant — NEVER STRETCHES!) ---
  const L_UPPER_ARM = 335 * scaleMm;
  const L_FOREARM = 305 * scaleMm;
  const gripBallX = upperVF.sliderX - 8 * scaleMm;
  const gripBallY = upperVF.sliderY + 4 * scaleMm;
  // Elbow bends DOWNWARD/BACKWARD (+1 in canvas Y-down coordinates)
  const armIK = solveTwoBoneIK(shoulderX, shoulderY, gripBallX, gripBallY, L_UPPER_ARM, L_FOREARM, +1);
  const elbowX = armIK.jointX;
  const elbowY = armIK.jointY;
  const handX = armIK.effX;
  const handY = armIK.effY;

  if (armIK.clamped) {
    ctx.strokeStyle = '#38bdf8';
    ctx.lineWidth = 2;
    ctx.setLineDash([3, 2]);
    ctx.beginPath();
    ctx.moveTo(gripBallX, gripBallY);
    ctx.lineTo(handX, handY);
    ctx.stroke();
    ctx.setLineDash([]);
  }

  const armBase = upperActive ? '#0284c7' : '#64748b';
  const armHi = upperActive ? '#7dd3fc' : '#cbd5e1';
  const armStroke = upperActive ? '#bae6fd' : '#94a3b8';
  const shR = Math.max(5.5, 24 * scaleMm);
  const elR = Math.max(4.6, 19 * scaleMm);
  const wrR = Math.max(3.6, 14 * scaleMm);

  // Draw Rigid Upper Arm & Rigid Forearm
  drawTaperedLimb(ctx, shoulderX, shoulderY, elbowX, elbowY, shR, elR, armBase, armHi, armStroke);
  drawTaperedLimb(ctx, elbowX, elbowY, handX, handY, elR * 0.92, wrR, armBase, armHi, armStroke);

  // Shoulder & Elbow Joint Caps
  [ [shoulderX, shoulderY], [elbowX, elbowY] ].forEach(([jx, jy]) => {
    ctx.fillStyle = upperActive ? '#0369a1' : '#475569';
    ctx.strokeStyle = upperActive ? '#e0f2fe' : '#cbd5e1';
    ctx.lineWidth = 1.4;
    ctx.beginPath();
    ctx.arc(jx, jy, 4.2, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
  });

  // Wireless EMG Sensor Band on Forearm/Wrist (with glowing green/red LED)
  const wristX = elbowX + (handX - elbowX) * 0.78;
  const wristY = elbowY + (handY - elbowY) * 0.78;
  const forearmAng = Math.atan2(handY - elbowY, handX - elbowX);
  ctx.save();
  ctx.translate(wristX, wristY);
  ctx.rotate(forearmAng);
  ctx.fillStyle = '#0f172a';
  ctx.strokeStyle = isHwEmergency ? emergencyRed : '#2dd4bf';
  ctx.lineWidth = 1.6;
  ctx.beginPath();
  ctx.roundRect(-5, -wrR - 2.5, 10, (wrR + 2.5) * 2, 3);
  ctx.fill();
  ctx.stroke();
  ctx.fillStyle = isHwEmergency ? emergencyRed : '#34d399';
  ctx.beginPath();
  ctx.arc(0, 0, 2.2, 0, Math.PI * 2);
  ctx.fill();
  ctx.restore();

  // 5. FOREGROUND STEEL V-FRAME ROBOTIC LINKAGES + VISIBLE MOTORS
  function renderSteelVMechanism(vf, thetaDeg, isUpper, isHighlighted, motorLabel) {
    ctx.save();

    // 5A. 680mm Main Extruded Steel Box Arm
    drawSteelBeam(ctx, vf.pivotX, vf.pivotY, vf.apexX, vf.apexY, isHighlighted ? 10 : 8, isHighlighted, false);

    // 5B. 370mm Slotted Linear Actuator Steel Rail with Visible Lead-Screw
    drawSteelBeam(ctx, vf.apexX, vf.apexY, vf.railEndX, vf.railEndY, isHighlighted ? 12 : 9.5, isHighlighted, true);

    // 5C. Bolted Steel Corner Gusset + Compact Linear Stepper Motor at V-Apex
    ctx.save();
    ctx.translate(vf.apexX, vf.apexY);
    ctx.rotate(-vf.rad2);
    // Linear motor box sitting on top of the 370mm rail apex
    ctx.fillStyle = '#1e293b';
    ctx.strokeStyle = isHighlighted ? '#94a3b8' : '#475569';
    ctx.lineWidth = 1.3;
    ctx.beginPath();
    ctx.roundRect(-14, -7.5, 15, 15, 3);
    ctx.fill();
    ctx.stroke();
    // Apex reinforcement bolt plate
    ctx.fillStyle = '#cbd5e1';
    ctx.strokeStyle = '#334155';
    ctx.lineWidth = 1.2;
    ctx.beginPath();
    ctx.arc(0, 0, 6.5, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = '#0f172a';
    ctx.beginPath();
    ctx.arc(0, 0, 2.5, 0, Math.PI * 2);
    ctx.fill();
    ctx.restore();

    // 5D. Machined Aluminum Linear Slider Carriage Block on the 370mm Rail
    ctx.save();
    ctx.translate(vf.sliderX, vf.sliderY);
    ctx.rotate(-vf.rad2);
    const carGrad = ctx.createLinearGradient(-9, -8, 9, 8);
    carGrad.addColorStop(0, '#e2e8f0');
    carGrad.addColorStop(0.5, '#64748b');
    carGrad.addColorStop(1, '#1e293b');
    ctx.fillStyle = carGrad;
    ctx.strokeStyle = isHwEmergency ? emergencyRed : (isHighlighted ? '#38bdf8' : '#94a3b8');
    ctx.lineWidth = 1.6;
    ctx.beginPath();
    ctx.roundRect(-9, -7.5, 18, 15, 3.5);
    ctx.fill();
    ctx.stroke();
    // 4 corner carriage bolts
    ctx.fillStyle = '#0f172a';
    [[-5.5, -4], [5.5, -4], [-5.5, 4], [5.5, 4]].forEach(([bx, by]) => {
      ctx.beginPath();
      ctx.arc(bx, by, 1.3, 0, Math.PI * 2);
      ctx.fill();
    });
    ctx.restore();

    // 5E. End-Effector: Blue Ergonomic Grip Ball (Upper) or Padded Foot/Ankle Sling (Lower)
    if (isUpper) {
      const bx = handX;
      const by = handY;
      const ballR = Math.max(8.5, 34 * scaleMm);
      const grad = ctx.createRadialGradient(bx - 3, by - 3, 1.5, bx, by, ballR);
      grad.addColorStop(0, '#bae6fd');
      grad.addColorStop(0.5, '#0ea5e9');
      grad.addColorStop(1, '#0369a1');
      ctx.fillStyle = grad;
      ctx.beginPath();
      ctx.arc(bx, by, ballR, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = '#e0f2fe';
      ctx.lineWidth = 1.2;
      ctx.stroke();
    } else {
      // Padded ankle cuff wrap around patient's ankle
      ctx.strokeStyle = isHighlighted ? '#34d399' : '#cbd5e1';
      ctx.lineWidth = 2.6;
      ctx.beginPath();
      ctx.roundRect(ankleX - 8, ankleY - 8, 16, 18, 5);
      ctx.stroke();
    }

    // 5F. Base Rotary Servo Motor (M1 / M2) with Rotating Gear Hub
    drawRotaryServoMotor(ctx, vf.pivotX, vf.pivotY, thetaDeg, isHighlighted, isHwEmergency, emergencyBlinkOn, motorLabel);

    ctx.restore();
  }

  renderSteelVMechanism(upperVF, k.upperTheta1, true, upperActive, 'M1 SERVO');
  renderSteelVMechanism(lowerVF, k.lowerTheta1, false, lowerActive, 'M2 SERVO');

  // 6. OVERLAY 1: BLUE SKETCH TRAJECTORY ARROWS (เปลี่ยนเป็นสีแดงกะพริบทันทีเมื่อเกิดเหตุฉุกเฉินจากเซนเซอร์จริงหรือขัดมอเตอร์!)
  if (cadOverlays.sketch) {
    if (kinSelectedPart === 'arm' || kinSelectedPart === 'all4') {
      const pStart = computeVFramePoints(leftPivotX, leftPivotY, k.armThetaMin, k.armSlideMin, scaleMm);
      const pEnd = computeVFramePoints(leftPivotX, leftPivotY, k.armThetaMax, k.armSlideMax, scaleMm);
      drawCurvedTrajectoryArrow(
        ctx,
        pStart.sliderX + 12, pStart.sliderY,
        (pStart.sliderX + pEnd.sliderX) * 0.5 - 10, Math.max(66, Math.min(pStart.sliderY, pEnd.sliderY) - 34),
        pEnd.sliderX - 18, pEnd.sliderY + 10,
        isHwEmergency ? emergencyRed : '#38bdf8',
        isHwEmergency ? '🚨 ฉุกเฉิน! หยุดวิถีแขนกลทันที' : 'วิถีกวาดแขน 680mm Arc ↶'
      );
    }

    if (kinSelectedPart === 'elbow' || kinSelectedPart === 'all4') {
      drawCanvasArrow(
        ctx,
        upperVF.railEndX + 20, upperVF.railEndY - 5,
        upperVF.apexX + 20, upperVF.apexY + 12,
        isHwEmergency ? emergencyRed : '#60a5fa',
        3.2,
        9
      );
      drawCadLabelPill(
        ctx,
        upperVF.apexX + 76,
        (upperVF.apexY + upperVF.railEndY) * 0.5,
        isHwEmergency ? '🚨 หยุดราง 370mm' : 'สไลด์ข้อศอก 370mm ↕',
        isHwEmergency ? emergencyRed : '#93c5fd'
      );
    }

    if (kinSelectedPart === 'leg' || kinSelectedPart === 'all4') {
      drawCanvasArrow(
        ctx,
        lowerVF.railEndX + 20, lowerVF.railEndY - 4,
        lowerVF.apexX + 18, lowerVF.apexY + 8,
        isHwEmergency ? emergencyRed : '#34d399',
        3.2,
        9
      );
      drawCadLabelPill(
        ctx,
        lowerVF.apexX + 68,
        lowerVF.apexY + 18,
        isHwEmergency ? '🚨 หยุดยกขา!' : 'ยกขา 680/370mm ↖',
        isHwEmergency ? emergencyRed : '#6ee7b7'
      );
    }

    if (kinSelectedPart === 'knee' || kinSelectedPart === 'all4') {
      drawCurvedTrajectoryArrow(
        ctx,
        lowerVF.railEndX + 26, lowerVF.railEndY + 6,
        lowerVF.railEndX + 20, lowerVF.apexY + 14,
        lowerVF.apexX - 20, lowerVF.apexY + 18,
        isHwEmergency ? emergencyRed : '#2dd4bf',
        isHwEmergency ? '🚨 ฉุกเฉิน! ล็อกเข่าทันที' : 'งอ-เหยียดข้อเข่า 90° ↰'
      );
    }
  }

  // Flashing red border inside CAD canvas when emergency is active
  if (isHwEmergency) {
    ctx.save();
    ctx.strokeStyle = emergencyBlinkOn ? 'rgba(239, 68, 68, 0.95)' : 'rgba(248, 113, 113, 0.35)';
    ctx.lineWidth = 6;
    ctx.strokeRect(3, 3, cw - 6, ch - 6);
    ctx.restore();
  }

  // 7. OVERLAY 2: CLEAN NON-OVERLAPPING CAD DIMENSIONS (680mm / 370mm)
  if (cadOverlays.dims) {
    const mid680X = (upperVF.pivotX + upperVF.apexX) * 0.5 - 30;
    const mid680Y = (upperVF.pivotY + upperVF.apexY) * 0.5 - 4;
    drawCadLabelPill(ctx, mid680X, mid680Y, '680mm', '#38bdf8', 'rgba(56,189,248,0.45)');

    const mid370X = (upperVF.apexX + upperVF.railEndX) * 0.5 + 34;
    const mid370Y = (upperVF.apexY + upperVF.railEndY) * 0.5 - 12;
    drawCadLabelPill(ctx, mid370X, mid370Y, '370mm', '#34d399', 'rgba(52,211,153,0.45)');
  }

  // 8. OVERLAY 3: CLEAN IK ANGLE & SLIDE PILLS (positioned so they never overlap)
  if (cadOverlays.vectors) {
    drawCadLabelPill(ctx, leftPivotX - 38, leftPivotY - 22, `θ₁=${Math.round(k.upperTheta1)}°`, '#22d3ee');
    drawCadLabelPill(ctx, upperVF.sliderX + 38, upperVF.sliderY + 14, `S₂=${Math.round(k.upperSlide2)}mm`, '#34d399');

    drawCadLabelPill(ctx, rightPivotX - 38, rightPivotY - 22, `θ₁=${Math.round(k.lowerTheta1)}°`, '#22d3ee');
    drawCadLabelPill(ctx, lowerVF.sliderX + 38, lowerVF.sliderY + 14, `S₂=${Math.round(k.lowerSlide2)}mm`, '#34d399');
  }

  ctx.restore();

  // Update live numeric readouts in the bottom canvas bar (using exact rigid IK angles!)
  const realUpperArmAng = Math.atan2(elbowY - shoulderY, elbowX - shoulderX);
  const realForearmAng = Math.atan2(handY - elbowY, handX - elbowX);
  const realShoulderDeg = Math.max(0, Math.min(180, Math.round(90 - (realUpperArmAng * 180 / Math.PI))));
  let elbowDiff = Math.abs((realForearmAng - realUpperArmAng) * 180 / Math.PI);
  if (elbowDiff > 180) elbowDiff = 360 - elbowDiff;
  const realElbowDeg = Math.max(35, Math.min(178, Math.round(180 - elbowDiff)));

  const realThighAng = Math.atan2(kneeY - hipY, kneeX - hipX);
  const realShinAng = Math.atan2(ankleY - kneeY, ankleX - kneeX);
  const realHipDeg = Math.max(0, Math.min(120, Math.round((-realThighAng * 180 / Math.PI))));
  let kneeDiff = Math.abs((realShinAng - realThighAng) * 180 / Math.PI);
  if (kneeDiff > 180) kneeDiff = 360 - kneeDiff;
  const realKneeDeg = Math.max(0, Math.min(140, Math.round(kneeDiff)));

  const displayTheta = (kinSelectedPart === 'leg' || kinSelectedPart === 'knee') ? k.lowerTheta1 : k.upperTheta1;
  const displaySlide = (kinSelectedPart === 'leg' || kinSelectedPart === 'knee') ? k.lowerSlide2 : k.upperSlide2;

  const elT1 = document.getElementById('readoutTheta1');
  const elS2 = document.getElementById('readoutSlide2');
  const elJA = document.getElementById('readoutJointAngle');
  const elTq = document.getElementById('readoutAiTorque');
  if (elT1) elT1.innerText = `${displayTheta.toFixed(1)}°`;
  if (elS2) elS2.innerText = `${Math.round(displaySlide)} mm`;
  if (elJA) {
    if (kinSelectedPart === 'arm' || kinSelectedPart === 'elbow') {
      elJA.innerText = `ไหล่ ${realShoulderDeg}° | ศอก ${realElbowDeg}°`;
    } else if (kinSelectedPart === 'leg' || kinSelectedPart === 'knee') {
      elJA.innerText = `สะโพก ${realHipDeg}° | เข่า ${realKneeDeg}°`;
    } else {
      elJA.innerText = `ศอก ${realElbowDeg}° | เข่า ${realKneeDeg}°`;
    }
  }
  if (elTq) elTq.innerText = `${k.dynamicTorque.toFixed(1)} N (${k.assistPct}%)`;
}

